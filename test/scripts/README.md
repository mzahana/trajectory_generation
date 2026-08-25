# MPC node-level test harnesses

ROS-level checks for `mpc_12state_node` that unit tests cannot cover, because
they are about *node behaviour over time* (rates, staleness, health reporting)
rather than the QP math.

Run them against a live node. Source ROS **and** the workspace overlay first, and
background only the node — a `source ... && ros2 run ... &` chain backgrounds the
whole compound and leaves the harness in an unsourced shell:

```bash
source /opt/ros/humble/setup.bash
source ~/shared_volume/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 run trajectory_generation mpc_12state_node --ros-args \
  -p mpc_rate:=20.0 -p mpc_window:=10 -p dt_pred:=0.1 \
  -p minimum_altitude:=-1.0 -p ref_timeout:=0.5 &
NODE_PID=$!
sleep 3
python3 test_t17_rate.py     # or test_t17_stale.py
kill $NODE_PID
```

## `test_t17_rate.py` — solve rate is decoupled from the reference rate

Publishes odometry at 30 Hz and a reference path at a deliberately slow **2 Hz**,
then measures the MPC output rate and reads `mpc/out/health`.

Before T1.7 the node solved inside the reference callback, so its output rate
equalled the reference rate. Passing requires the output rate to track
`mpc_rate`, not the reference.

Observed: **20.0 Hz output from a 2 Hz reference**, max solve 0.98 ms,
150 solves / 0 failures.

## `test_t17_stale.py` — stale reference stops the setpoint stream

Runs healthy, then **stops publishing the reference** and verifies the node
(a) emits no further setpoints and (b) reports ERROR-level health.

This is a safety property: solving on a stale reference would feed the geometric
controller a confident-looking setpoint derived from a target that may no longer
be there. Stopping instead lets the controller's hold-on-setpoint-loss failsafe
take over.

Observed: 116 setpoints while fresh → **0 while stale**, health level 2 (ERROR),
message `reference trajectory stale`.
