# Project memory

## Physical robot calibration

- Keep the intentional wheelbase mismatch unchanged for now:
  - `serial_bridge.py` and `APFController` use `0.45 m`.
  - `encoder_odom.py` uses `0.8 m`.
- This is an on-vehicle compensation for wheel slip, not an accidental inconsistency.
- The IMU has not yet been purchased or integrated. Revisit this calibration only after IMU-based state estimation/fusion is available; do not "fix" or normalize these values beforehand.

## Execution environment

- This computer is a code-editing workspace only. Do not expect ROS builds, simulation, or hardware tests to run locally.
- The project is built, run, and validated on a remote machine. Treat local static checks as preflight validation and leave runtime verification for the remote environment.
