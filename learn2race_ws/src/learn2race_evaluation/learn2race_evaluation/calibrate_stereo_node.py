#!/usr/bin/env python3
"""Headless stereo calibration for the IMX219 pair (no GUI, runs on the robot).

The ROS camera_calibration GUI needs a display, and streaming two raw image
pairs to a desktop over Wi-Fi is too slow for it. This node does the same job
on the robot: it watches both cameras, captures a pair whenever the checkerboard
is seen sharply by both in a pose that adds something new, and then solves

  * each camera's intrinsics (cv2.calibrateCamera, plumb_bob distortion),
  * the right camera's pose relative to the left (cv2.stereoCalibrate, with the
    intrinsics fixed), and
  * the rectification (cv2.stereoRectify),

writing ROS camera_info YAML files that camera_ros loads through
camera_info_url and stereo_image_proc uses for rectification and disparity.

    ros2 launch learn2race_bringup stereo_capture.launch.py
    ros2 run learn2race_evaluation calibrate_stereo

Captured pairs are saved, so the solve can be repeated without the board:

    ros2 run learn2race_evaluation calibrate_stereo --ros-args -p from_images:=true
"""

# References (full entries: docs/references.md at the repository root):
#   [R34] Zhang (2000) camera calibration: calibration
#   [R30] OpenCV 4.x: cv2.stereoCalibrate, findChessboardCorners
#   [R66] Sony IMX219: IMX219
# More details: README.md of this package

import glob
import math
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

FIND_FLAGS = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
              | cv2.CALIB_CB_FAST_CHECK)
SUBPIX = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def board_pose(corners, width, height, cols, rows):
    """Describe a view by centre x/y, apparent size and skew, each in 0..1."""
    pts = corners.reshape(-1, 2)
    x, y = pts[:, 0].mean() / width, pts[:, 1].mean() / height
    outer = np.array([pts[0], pts[cols - 1], pts[-1], pts[-cols]], dtype=np.float32)
    area = cv2.contourArea(outer) / (width * height)
    size = math.sqrt(max(area, 0.0))
    # Skew: how far the top edge departs from parallel to the bottom edge.
    top, bottom = outer[1] - outer[0], outer[2] - outer[3]
    cosang = float(np.dot(top, bottom) / (np.linalg.norm(top) * np.linalg.norm(bottom) + 1e-9))
    skew = min(1.0, math.acos(max(-1.0, min(1.0, cosang))) / (math.pi / 4))
    return np.array([x, y, size, skew])


def first_square_sign(gray, corners, cols):
    """+1 if the square after the first corner is lighter than its neighbour.

    A 10x7-corner board looks the same rotated 180 degrees, so OpenCV may
    number an upside-down view from the opposite end. On a board with an odd
    and an even number of squares the two ends have different colours, so the
    colour of the first square tells which physical end corner 0 is.
    """
    pts = corners.reshape(-1, 2)
    first = (pts[0] + pts[1] + pts[cols] + pts[cols + 1]) / 4
    second = (pts[1] + pts[2] + pts[cols + 1] + pts[cols + 2]) / 4
    sample = lambda p: float(gray[int(round(p[1])), int(round(p[0]))])
    return 1 if sample(first) > sample(second) else -1


def yaml_matrix(name, rows, cols, values):
    data = ', '.join(f'{v:.8f}' for v in np.asarray(values).ravel())
    return f'{name}:\n  rows: {rows}\n  cols: {cols}\n  data: [{data}]\n'


def write_camera_info(path, name, width, height, K, D, R, P):
    with open(path, 'w') as f:
        f.write(f'image_width: {width}\nimage_height: {height}\ncamera_name: {name}\n')
        f.write(yaml_matrix('camera_matrix', 3, 3, K))
        f.write('distortion_model: plumb_bob\n')
        f.write(yaml_matrix('distortion_coefficients', 1, 5, np.asarray(D).ravel()[:5]))
        f.write(yaml_matrix('rectification_matrix', 3, 3, R))
        f.write(yaml_matrix('projection_matrix', 3, 4, P))


class CalibrateStereo(Node):

    def __init__(self):
        super().__init__('calibrate_stereo')
        self.declare_parameter('left_topic', '/stereo/left/image_raw')
        self.declare_parameter('right_topic', '/stereo/right/image_raw')
        # Default: the lab's printed board, 11x8 squares of 20 mm = 10x7
        # INNER corners. Hold it landscape (10 corners across). Measure a
        # printed square: a scaled print biases every focal length.
        self.declare_parameter('board_cols', 10)           # inner corners across
        self.declare_parameter('board_rows', 7)            # inner corners down
        self.declare_parameter('square_m', 0.020)
        self.declare_parameter('target_pairs', 30)
        self.declare_parameter('min_novelty', 0.12)        # pose distance to accept
        self.declare_parameter('output_dir', '~/learn2race/config/stereo')
        self.declare_parameter('from_images', False)
        p = lambda n: self.get_parameter(n).value
        self.cols, self.rows = int(p('board_cols')), int(p('board_rows'))
        self.square = float(p('square_m'))
        self.target = int(p('target_pairs'))
        self.min_novelty = float(p('min_novelty'))
        self.out = os.path.expanduser(str(p('output_dir')))
        self.image_dir = os.path.join(self.out, 'images')
        os.makedirs(self.image_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.pairs = []            # (left_corners, right_corners)
        self.poses = []
        self.size = None
        self.last_left = None
        self.busy = False
        self.last_status = 0.0
        self.done = False

        if bool(p('from_images')):
            threading.Thread(target=self._from_images, daemon=True).start()
            return
        for old in glob.glob(os.path.join(self.image_dir, '*.png')):
            os.remove(old)
        left = Subscriber(self, Image, p('left_topic'), qos_profile=qos_profile_sensor_data)
        right = Subscriber(self, Image, p('right_topic'), qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([left, right], queue_size=5, slop=0.05)
        self.sync.registerCallback(self._on_pair)
        print(f'\nStereo calibration: {self.cols}x{self.rows} inner corners, '
              f'{self.square * 1000:.0f} mm squares, {self.target} pairs.')
        print('Hold the board so BOTH cameras see all of it. Keep it still for a '
              'moment at each pose; vary distance, position, and tilt.\n')

    # ------------------------------------------------------------- capture

    def _gray(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg)
        if image.ndim == 3:
            code = cv2.COLOR_BGRA2GRAY if image.shape[2] == 4 else cv2.COLOR_BGR2GRAY
            return image, cv2.cvtColor(image, code)
        return image, image

    def _find(self, gray):
        found, corners = cv2.findChessboardCorners(gray, (self.cols, self.rows), FIND_FLAGS)
        if not found:
            return None
        return cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), SUBPIX)

    def _on_pair(self, left_msg, right_msg):
        if self.busy or self.done:
            return
        self.busy = True
        try:
            self._handle(left_msg, right_msg)
        finally:
            self.busy = False

    def _handle(self, left_msg, right_msg):
        left_img, left_gray = self._gray(left_msg)
        right_img, right_gray = self._gray(right_msg)
        self.size = (left_gray.shape[1], left_gray.shape[0])
        lc, rc = self._find(left_gray), self._find(right_gray)
        now = time.monotonic()

        state = f'left {"OK" if lc is not None else "--"}  right {"OK" if rc is not None else "--"}'
        if lc is None or rc is None:
            self.last_left = None
            self._status(state, now)
            return
        if first_square_sign(left_gray, lc, self.cols) != first_square_sign(right_gray, rc, self.cols):
            self._status(state + '  CAMERAS DIFFER BY 180 deg: fix left/right_orientation', now)
            return
        # Accept only a still board: blur ruins sub-pixel corners.
        moving = (self.last_left is None or
                  float(np.abs(self.last_left - lc).mean()) > 1.0)
        self.last_left = lc
        if moving:
            self._status(state + '  hold still', now)
            return
        pose = board_pose(lc, *self.size, self.cols, self.rows)
        novelty = min((float(np.abs(pose - q).sum()) for q in self.poses), default=1.0)
        if novelty < self.min_novelty:
            self._status(state + '  move to a new pose', now)
            return

        self.pairs.append((lc, rc))
        self.poses.append(pose)
        n = len(self.pairs)
        cv2.imwrite(os.path.join(self.image_dir, f'left_{n:02d}.png'), left_img)
        cv2.imwrite(os.path.join(self.image_dir, f'right_{n:02d}.png'), right_img)
        print(f'\r  captured {n:2d}/{self.target}   {self._coverage()}' + ' ' * 10)
        if n >= self.target:
            self.done = True
            threading.Thread(target=self._solve, daemon=True).start()

    def _coverage(self):
        poses = np.array(self.poses)
        span = poses.max(axis=0) - poses.min(axis=0) if len(poses) > 1 else np.zeros(4)
        # Useful spreads: x, y across most of the image, size and skew varied.
        goals = np.array([0.6, 0.6, 0.3, 0.5])
        parts = []
        for label, value, goal in zip(('X', 'Y', 'size', 'skew'), span, goals):
            filled = int(round(10 * min(1.0, value / goal)))
            parts.append(f'{label} [{"#" * filled}{"." * (10 - filled)}]')
        return '  '.join(parts)

    def _status(self, text, now):
        if now - self.last_status > 0.5:
            self.last_status = now
            print(f'\r  {len(self.pairs):2d}/{self.target}  {text}' + ' ' * 20, end='', flush=True)

    # --------------------------------------------------------------- solve

    def _from_images(self):
        lefts = sorted(glob.glob(os.path.join(self.image_dir, 'left_*.png')))
        for left_path in lefts:
            right_path = left_path.replace('left_', 'right_')
            lg = cv2.imread(left_path, cv2.IMREAD_GRAYSCALE)
            rg = cv2.imread(right_path, cv2.IMREAD_GRAYSCALE)
            if lg is None or rg is None:
                continue
            self.size = (lg.shape[1], lg.shape[0])
            lc, rc = self._find(lg), self._find(rg)
            if lc is None or rc is None:
                continue
            if first_square_sign(lg, lc, self.cols) != first_square_sign(rg, rc, self.cols):
                flipped = getattr(self, 'flipped', 0) + 1
                self.flipped = flipped
                continue
            self.pairs.append((lc, rc))
        if getattr(self, 'flipped', 0):
            print(f'Skipped {self.flipped} pair(s) whose images differ by 180 degrees.')
        print(f'Loaded {len(self.pairs)} usable pairs from {self.image_dir}')
        self._solve()

    def _solve(self):
        if len(self.pairs) < 10:
            print(f'\nOnly {len(self.pairs)} pairs; at least 10 are needed.')
            rclpy.try_shutdown()
            return
        print('\nSolving (this takes a few seconds)...')
        obj = np.zeros((self.rows * self.cols, 3), np.float32)
        obj[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2) * self.square
        objs = [obj] * len(self.pairs)
        lefts = [p[0] for p in self.pairs]
        rights = [p[1] for p in self.pairs]

        # k3 is fixed at zero: on a ~62 degree lens it is poorly observed
        # (the board seldom reaches the corners of both views) and a free k3
        # soaks up error that then distorts the focal lengths and the pose.
        mono = cv2.CALIB_FIX_K3
        stereo = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_K3
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
        for attempt in (1, 2):
            rms_l, K1, D1, _, _ = cv2.calibrateCamera(objs, lefts, self.size, None, None, flags=mono)
            rms_r, K2, D2, _, _ = cv2.calibrateCamera(objs, rights, self.size, None, None, flags=mono)
            # Refine both cameras jointly with their relative pose.
            rms_s, K1, D1, K2, D2, R, T, _, _, errors = cv2.stereoCalibrateExtended(
                objs, lefts, rights, K1, D1, K2, D2, self.size, None, None,
                flags=stereo, criteria=criteria)
            if attempt == 2:
                break
            # Drop pairs far worse than the rest (blur, a bent board, a bad
            # detection), then solve again once.
            per_pair = np.asarray(errors).max(axis=1).ravel()
            keep = per_pair <= max(2.0 * float(np.median(per_pair)), 1.0)
            if keep.all() or keep.sum() < 10:
                break
            print(f'  dropping {int((~keep).sum())} outlier pair(s) of {len(keep)}')
            objs = [o for o, k in zip(objs, keep) if k]
            lefts = [c for c, k in zip(lefts, keep) if k]
            rights = [c for c, k in zip(rights, keep) if k]
        R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(
            K1, D1, K2, D2, self.size, R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)

        width, height = self.size
        write_camera_info(os.path.join(self.out, 'left.yaml'), 'stereo_left',
                          width, height, K1, D1, R1, P1)
        write_camera_info(os.path.join(self.out, 'right.yaml'), 'stereo_right',
                          width, height, K2, D2, R2, P2)

        baseline = float(np.linalg.norm(T))
        print('\n' + '=' * 60)
        print(f'  reprojection RMS  left {rms_l:.3f} px   right {rms_r:.3f} px   '
              f'stereo {rms_s:.3f} px')
        print(f'  focal length      left {K1[0, 0]:.1f} px   right {K2[0, 0]:.1f} px')
        print(f'  baseline          {baseline * 1000:.1f} mm   '
              f'(T = {T.ravel()[0] * 1000:+.1f}, {T.ravel()[1] * 1000:+.1f}, '
              f'{T.ravel()[2] * 1000:+.1f} mm)')
        print('=' * 60)
        if T.ravel()[0] > 0:
            print('  WARNING: the "right" camera is LEFT of the "left" one. Swap them:')
            print('  stereo_capture.launch.py left_camera:=1 right_camera:=0, then recalibrate.')
        if max(rms_l, rms_r) > 0.5 or rms_s > 1.0:
            print('  Error is high: check the printed square size, keep the board flat,')
            print('  and hold it still. Under 0.3 px per camera is good.')
        print(f'  Measure the distance between the two lens centres: it should match')
        print(f'  the baseline above within a few millimetres.')
        print(f'\nWritten {self.out}/left.yaml and right.yaml.')
        print('Restart stereo_capture.launch.py to load them.')
        rclpy.try_shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateStereo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
