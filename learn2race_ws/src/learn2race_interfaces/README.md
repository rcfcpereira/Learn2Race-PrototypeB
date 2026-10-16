# learn2race_interfaces

Custom ROS 2 action interfaces for Learn2Race autonomous racing and sensor calibration workflows.

## Actions

### DragsterRules

Competition-style dragster run contract with lane discipline, timing, speed, and telemetry feedback.

**Goal:**
- Race configuration (race ID, lane ID, autonomous-only mode)
- Timing constraints (max run time, start delay)
- Performance limits (speed, acceleration, deceleration, lane deviation)
- Allowed control modes

**Result:**
- Success status and accepted flag
- Final run telemetry (elapsed time, top speed, distance, average speed)
- Rule violation report
- Final status string

**Feedback:**
- Live telemetry: current speed, distance, elapsed time
- Real-time constraint monitoring: lateral deviation, active rule violations
- State tracking: current phase/state

### StartCameraLineDetection

Startup contract for camera and line detection pipeline with black-line calibration and preview progress.

**Goal:**
- Pipeline control flags (start camera, start line detection, request preview)
- Media configuration (image topic, encoding, frame ID)
- Detection parameters (line color, HSV detection toggle, calibration mode, thresholds, ROI ratios)

**Result:**
- Success status and pipeline readiness flags
- Detection capability report (line detected, calibration complete, preview started)
- Effective mode string and status message

**Feedback:**
- State: initialization, calibration, tracking, ready
- Detection quality: line error, confidence, detection threshold
- Calibration progress (0.0 to 1.0)

## Notes

- Both actions use only builtin ROS 2 interface types (`builtin_interfaces/Duration`, standard msgs) for maximum portability.
- `DragsterRules` uses ROS `builtin_interfaces/Duration` for timing constraints instead of raw float seconds.
- Actions are designed for both synchronous setup (initial calibration) and asynchronous feedback loops (runtime monitoring).
- All feedback fields are optional; clients should handle partial state gracefully.

## References and more details

- [R10] ROS 2 interfaces (messages, actions); [R2] Dragster rules.
- Full entries: [`docs/references.md`](../../../docs/references.md).
