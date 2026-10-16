#!/usr/bin/env python3
"""Check stereo depth against a tape measure.

Point the cameras at a flat, textured surface (a wall with a poster, a box)
at a measured distance. The node prints the median depth in a centre window
of the disparity image, Z = f * B / d, and saves a colour-coded disparity image
so gaps and noise can be inspected.

    ros2 launch learn2race_bringup stereo_capture.launch.py depth:=true
    ros2 run learn2race_evaluation stereo_depth_check
"""

# References (full entries: docs/references.md at the repository root):
#   [R35] Hartley & Zisserman, Multiple View Geometry: depth from disparity, Z = f*B/d
#   [R30] OpenCV 4.x: StereoSGBM
# More details: README.md of this package

import os

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from stereo_msgs.msg import DisparityImage


class StereoDepthCheck(Node):

    def __init__(self):
        super().__init__('stereo_depth_check')
        self.declare_parameter('topic', '/stereo/disparity')
        self.declare_parameter('window', 0.15)          # centre window, fraction of width
        self.declare_parameter('snapshot', '~/learn2race/config/stereo/disparity_check.png')
        self.bridge = CvBridge()
        self.window = float(self.get_parameter('window').value)
        self.snapshot = os.path.expanduser(str(self.get_parameter('snapshot').value))
        self.count = 0
        self.create_subscription(DisparityImage, self.get_parameter('topic').value,
                                 self._on_disparity, 5)
        print('Waiting for /stereo/disparity (launch with depth:=true)...')

    def _on_disparity(self, msg):
        self.count += 1
        if self.count % 5:                               # a few prints per second
            return
        disp = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='32FC1')
        h, w = disp.shape
        half = max(4, int(w * self.window / 2))
        patch = disp[h // 2 - half:h // 2 + half, w // 2 - half:w // 2 + half]
        valid_all = disp > msg.min_disparity
        valid = patch[patch > msg.min_disparity]
        fill = 100.0 * valid_all.mean()
        if valid.size < patch.size * 0.2:
            print(f'\r  centre: too few matches ({valid.size}/{patch.size}); aim at '
                  f'textured surface   | image filled {fill:4.1f}%   ', end='', flush=True)
        else:
            d = float(np.median(valid))
            z = msg.f * msg.t / d
            spread = msg.f * msg.t / np.percentile(valid, 25) - msg.f * msg.t / np.percentile(valid, 75)
            print(f'\r  centre depth {z:6.3f} m  (+/-{spread / 2:.3f})  disparity {d:5.1f} px '
                  f'| image filled {fill:4.1f}%   ', end='', flush=True)
        # Colour snapshot: near = red, far = blue, no match = black.
        shown = np.clip((disp - msg.min_disparity) / max(msg.max_disparity - msg.min_disparity, 1),
                        0, 1)
        colour = cv2.applyColorMap((shown * 255).astype(np.uint8), cv2.COLORMAP_JET)
        colour[~valid_all] = 0
        cv2.rectangle(colour, (w // 2 - half, h // 2 - half), (w // 2 + half, h // 2 + half),
                      (255, 255, 255), 1)
        cv2.imwrite(self.snapshot, colour)


def main(args=None):
    rclpy.init(args=args)
    node = StereoDepthCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
