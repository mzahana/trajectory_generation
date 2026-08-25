#!/usr/bin/env python3
"""T1.8: SITL flight regression of the Phase-1 MPC fixes.

Flies the full chain (reference -> MPC -> geometric controller -> PX4) against a
constant-velocity target and measures how well the interceptor tracks it.

Sequence:
  1. Publish a stationary reference at takeoff altitude; the MPC turns it into a
     climb command. Enable motors, arm, switch to OFFBOARD.
  2. Once at altitude, switch the reference to a constant-velocity target.
  3. Record interceptor odom vs the reference and report steady-state along-track
     lag, cross-track error, and stability indicators.

Usage:
  python3 test_t18_sitl_tracking.py [--speed 2.0] [--alt 10.0] [--label fixed]

Requires SITL up (sitl_bringup.launch.py with_controller:=true) and an
mpc_12state_node running with its topics remapped to this vehicle.
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

NS = '/interceptor'
N = 10          # mpc_window
DT = 0.1        # dt_pred


class T18(Node):
    def __init__(self, speed, alt):
        super().__init__('t18_sitl_tracking')
        self.speed = speed
        self.alt = alt

        self.path_pub = self.create_publisher(Path, '/mpc/in/ref_traj_path', qos_profile_sensor_data)
        self.motors_pub = self.create_publisher(Bool, f'{NS}/geometric_controller/enable_motors', 10)

        self.create_subscription(Odometry, f'{NS}/mavros/local_position/odom',
                                 self.on_odom, qos_profile_sensor_data)
        self.create_subscription(State, f'{NS}/mavros/state', self.on_state, 10)

        self.arm_cli = self.create_client(CommandBool, f'{NS}/mavros/cmd/arming')
        self.mode_cli = self.create_client(SetMode, f'{NS}/mavros/set_mode')

        self.odom = None
        self.state = None

        # Reference target state, updated by the phase logic.
        self.target = [0.0, 0.0, alt]
        self.target_vel = [0.0, 0.0, 0.0]

        self.samples = []       # (t, target_xyz, interceptor_xyz, interceptor_v)
        self.recording = False

        self.create_timer(1.0 / 20.0, self.pub_ref)

    def on_odom(self, m):
        self.odom = m

    def on_state(self, m):
        self.state = m

    def pub_ref(self):
        """Publish the target's predicted path: constant-velocity extrapolation."""
        p = Path()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = f'{NS}/odom'
        for i in range(N + 1):
            tau = i * DT
            ps = PoseStamped()
            ps.header = p.header
            ps.pose.position.x = self.target[0] + self.target_vel[0] * tau
            ps.pose.position.y = self.target[1] + self.target_vel[1] * tau
            ps.pose.position.z = self.target[2] + self.target_vel[2] * tau
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        self.path_pub.publish(p)

        if self.recording and self.odom is not None:
            o = self.odom.pose.pose.position
            v = self.odom.twist.twist.linear
            self.samples.append((time.time(),
                                 tuple(self.target),
                                 (o.x, o.y, o.z),
                                 (v.x, v.y, v.z)))


def spin_for(node, seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.02)


def call(node, client, req, timeout=5.0):
    if not client.wait_for_service(timeout_sec=timeout):
        return None
    fut = client.call_async(req)
    t0 = time.time()
    while rclpy.ok() and not fut.done() and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
    return fut.result() if fut.done() else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=2.0, help='target speed, m/s')
    ap.add_argument('--alt', type=float, default=10.0, help='takeoff altitude, m')
    ap.add_argument('--label', type=str, default='run')
    ap.add_argument('--track-time', type=float, default=20.0)
    args = ap.parse_args()

    rclpy.init()
    n = T18(args.speed, args.alt)

    print(f'[{args.label}] waiting for odom/state ...')
    t0 = time.time()
    while (n.odom is None or n.state is None) and time.time() - t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.odom is None or n.state is None:
        print('FAIL: no odom/state'); return 1

    start_xy = (n.odom.pose.pose.position.x, n.odom.pose.pose.position.y)
    n.target = [start_xy[0], start_xy[1], args.alt]

    # Stream the reference so the MPC produces setpoints before OFFBOARD.
    print(f'[{args.label}] streaming reference, enabling motors ...')
    spin_for(n, 2.0)
    for _ in range(5):
        n.motors_pub.publish(Bool(data=True)); spin_for(n, 0.1)

    print(f'[{args.label}] arming ...')
    r = call(n, n.arm_cli, CommandBool.Request(value=True))
    print(f'  arm result: {r}')
    spin_for(n, 1.0)

    print(f'[{args.label}] OFFBOARD ...')
    r = call(n, n.mode_cli, SetMode.Request(base_mode=0, custom_mode='OFFBOARD'))
    print(f'  mode result: {r}')

    # Climb.
    print(f'[{args.label}] climbing to {args.alt} m ...')
    t0 = time.time()
    while time.time() - t0 < 40:
        rclpy.spin_once(n, timeout_sec=0.05)
        z = n.odom.pose.pose.position.z
        if abs(z - args.alt) < 1.0:
            break
    z = n.odom.pose.pose.position.z
    print(f'  altitude {z:.2f} m, mode={n.state.mode}, armed={n.state.armed}')
    if abs(z - args.alt) > 2.0:
        print(f'FAIL: did not reach altitude (z={z:.2f})')
        return 1

    spin_for(n, 3.0)   # settle

    # Constant-velocity target along +x.
    print(f'[{args.label}] target moving at {args.speed} m/s for {args.track_time}s ...')
    n.target_vel = [args.speed, 0.0, 0.0]
    n.recording = True
    t_start = time.time()
    last = t_start
    while time.time() - t_start < args.track_time:
        rclpy.spin_once(n, timeout_sec=0.02)
        now = time.time()
        n.target[0] += args.speed * (now - last)
        last = now
    n.recording = False

    # --- metrics -----------------------------------------------------------
    # Steady state = last 60% of the run.
    s = n.samples[int(len(n.samples) * 0.4):]
    if not s:
        print('FAIL: no samples'); return 1

    along = [t[0] - i[0] for _, t, i, _ in s]         # target x - interceptor x
    cross = [abs(t[1] - i[1]) for _, t, i, _ in s]
    alt_err = [abs(t[2] - i[2]) for _, t, i, _ in s]
    vx = [v[0] for _, _, _, v in s]

    def mean(a): return sum(a) / len(a)

    along_m = mean(along)
    lag_s = along_m / args.speed if args.speed > 0 else float('nan')
    # Oscillation indicator: std-dev of along-track error in steady state.
    along_std = math.sqrt(mean([(a - along_m) ** 2 for a in along]))

    print(f'\n--- T1.8 [{args.label}] speed={args.speed} m/s ---')
    print(f'samples                 : {len(s)}')
    print(f'along-track error (mean): {along_m:.2f} m  -> lag {lag_s:.2f} s')
    print(f'along-track error (std) : {along_std:.2f} m   (oscillation indicator)')
    print(f'cross-track error (mean): {mean(cross):.2f} m')
    print(f'altitude error (mean)   : {mean(alt_err):.2f} m')
    print(f'interceptor vx (mean)   : {mean(vx):.2f} m/s (body-frame odom twist)')
    print(f'final mode/armed        : {n.state.mode}/{n.state.armed}')

    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
