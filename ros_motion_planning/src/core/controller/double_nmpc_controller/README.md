# Double NMPC local controller

`double_nmpc_controller/DoubleNMPCController` is a new `move_base` local-planner plugin.
It leaves the existing APF controller and the physical wheelbase compensation unchanged.

The controller has two nonlinear rollout layers:

- A low-rate planner layer refreshes a long look-ahead reference every `0.48 s`.
- A high-rate tracking layer searches safe velocity/angular-rate rollouts every `0.16 s`.

Tracking position error, heading error, and tracking-layer correction form an EWMA risk.
The risk changes only the obstacle-clearance margin. The margin is allowed to relax to
`0.05 m`; it may tighten to `0.10 m` only when an obstacle is relevant to the current
costmap rollout. The plugin publishes `~/tracking_risk` and `~/dynamic_safe_margin`.

To test it, change the local planner in a dedicated launch file—not the existing debug
entrypoint—and load the supplied parameters:

```xml
<param name="base_local_planner" value="double_nmpc_controller/DoubleNMPCController"/>
<rosparam command="load"
          file="$(find double_nmpc_controller)/config/double_nmpc_high_speed.yaml"
          ns="DoubleNMPCController"/>
```

The supplied configuration starts at `0.40 m/s`, but it must first be validated with the
robot lifted and then in an open, obstacle-free area. Do not change the calibrated
`serial_bridge.py` (`0.45 m`) or `encoder_odom.py` (`0.8 m`) wheelbase values.
