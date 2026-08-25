#!/usr/bin/env python3
"""Speed sweep: find where the current pure-pursuit chain breaks down.

Takes off once, then steps the target's speed upward. At each step the target is
re-anchored at the interceptor's current position (so every step starts from a
converged state) and flown at constant velocity while the steady-state
along-track error is measured.

Pure pursuit converges only if the interceptor can match the target's speed.
The break point is the speed at which steady-state error stops settling and
instead grows without bound.

Usage: python3 test_speed_sweep.py --speeds 2,5,8,12,16 --seg 14
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
N = 10
DT = 0.1


class Sweep(Node):
    def __init__(self, alt):
        super().__init__('speed_sweep')
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
        self.target = [0.0, 0.0, alt]
        self.tvel = [0.0, 0.0, 0.0]
        self.rec = []
        self.recording = False
        self.create_timer(1.0 / 20.0, self.pub_ref)

    def on_odom(self, m): self.odom = m
    def on_state(self, m): self.state = m

    def ipos(self):
        p = self.odom.pose.pose.position
        return (p.x, p.y, p.z)

    def pub_ref(self):
        p = Path()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = f'{NS}/odom'
        for i in range(N + 1):
            tau = i * DT
            ps = PoseStamped()
            ps.header = p.header
            ps.pose.position.x = self.target[0] + self.tvel[0] * tau
            ps.pose.position.y = self.target[1] + self.tvel[1] * tau
            ps.pose.position.z = self.target[2] + self.tvel[2] * tau
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        self.path_pub.publish(p)
        if self.recording and self.odom is not None:
            ip = self.ipos()
            self.rec.append((time.time(), tuple(self.target), ip))


def spin_for(n, s):
    t0 = time.time()
    while time.time() - t0 < s:
        rclpy.spin_once(n, timeout_sec=0.02)


def call(n, cli, req, timeout=5.0):
    if not cli.wait_for_service(timeout_sec=timeout):
        return None
    f = cli.call_async(req)
    t0 = time.time()
    while rclpy.ok() and not f.done() and time.time() - t0 < timeout:
        rclpy.spin_once(n, timeout_sec=0.05)
    return f.result() if f.done() else None


def mean(a): return sum(a) / len(a) if a else float('nan')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speeds', type=str, default='2,5,8,12,16')
    ap.add_argument('--seg', type=float, default=14.0, help='seconds per speed step')
    ap.add_argument('--alt', type=float, default=15.0)
    args = ap.parse_args()
    speeds = [float(s) for s in args.speeds.split(',')]

    rclpy.init()
    n = Sweep(args.alt)

    t0 = time.time()
    while (n.odom is None or n.state is None) and time.time() - t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.odom is None:
        print('FAIL: no odom'); return 1

    ip = n.ipos()
    n.target = [ip[0], ip[1], args.alt]
    spin_for(n, 2.0)
    for _ in range(5):
        n.motors_pub.publish(Bool(data=True)); spin_for(n, 0.1)
    call(n, n.arm_cli, CommandBool.Request(value=True))
    spin_for(n, 1.0)
    call(n, n.mode_cli, SetMode.Request(base_mode=0, custom_mode='OFFBOARD'))

    print(f'climbing to {args.alt} m ...')
    t0 = time.time()
    while time.time() - t0 < 45:
        rclpy.spin_once(n, timeout_sec=0.05)
        if abs(n.ipos()[2] - args.alt) < 1.0:
            break
    print(f'  z={n.ipos()[2]:.2f} mode={n.state.mode} armed={n.state.armed}')
    spin_for(n, 3.0)

    print(f'\n{"speed":>7} {"lag_mean":>9} {"lag_end":>9} {"growth":>9} {"verdict":>12}')
    print('-' * 52)
    results = []
    for sp in speeds:
        # Re-anchor the target on the interceptor so each step starts converged.
        ip = n.ipos()
        n.target = [ip[0], ip[1], args.alt]
        n.tvel = [sp, 0.0, 0.0]
        n.rec = []
        n.recording = True
        t_start = time.time(); last = t_start
        while time.time() - t_start < args.seg:
            rclpy.spin_once(n, timeout_sec=0.02)
            now = time.time()
            n.target[0] += sp * (now - last)
            last = now
        n.recording = False

        s = n.rec
        if not s:
            continue
        err = [t[0] - i[0] for _, t, i in s]
        half = len(err) // 2
        first_half = mean(err[:half])
        second_half = mean(err[half:])
        growth = second_half - first_half          # >0 and large => diverging
        lag_end = mean(err[-max(1, len(err)//5):])
        # Converged if the error stops growing appreciably over the segment.
        diverging = growth > 0.15 * args.seg * 0.5   # ~ >0.5 m/s of sustained growth
        verdict = 'DIVERGING' if diverging else 'converged'
        results.append((sp, second_half, lag_end, growth, verdict))
        print(f'{sp:7.1f} {second_half:9.2f} {lag_end:9.2f} {growth:9.2f} {verdict:>12}')

    print('\nInterpretation: "converged" = interceptor holds a bounded trailing')
    print('distance. "DIVERGING" = it is being outrun; the gap grows without bound.')

    n.recording = False
    n.tvel = [0.0, 0.0, 0.0]
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
