#!/usr/bin/env python3
"""T1.7 acceptance check: MPC solve rate is decoupled from the reference rate.

Drives mpc_12state_node with odometry at 30 Hz and a reference path at a
deliberately slow 2 Hz, then measures the rate of the MPC's own output and reads
its health topic. Before T1.7 the node solved inside the reference callback, so
the output rate would equal the reference rate (2 Hz).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from custom_trajectory_msgs.msg import StateTrajectory
from diagnostic_msgs.msg import DiagnosticStatus
import time

REF_HZ = 2.0          # deliberately slow reference
ODOM_HZ = 30.0
MEASURE_S = 6.0
N = 10                # must match mpc_window


class Harness(Node):
    def __init__(self):
        super().__init__('t17_harness')
        self.odom_pub = self.create_publisher(Odometry, 'mpc/in/odom', qos_profile_sensor_data)
        self.path_pub = self.create_publisher(Path, 'mpc/in/ref_traj_path', qos_profile_sensor_data)
        self.create_subscription(StateTrajectory, 'mpc/out/trajectory', self.on_traj, 10)
        self.create_subscription(DiagnosticStatus, 'mpc/out/health', self.on_health, 10)

        self.traj_times = []
        self.health = []
        self.t0 = time.time()

        self.create_timer(1.0 / ODOM_HZ, self.pub_odom)
        self.create_timer(1.0 / REF_HZ, self.pub_ref)

    def pub_odom(self):
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_link'
        m.pose.pose.position.z = 10.0
        m.pose.pose.orientation.w = 1.0
        m.twist.twist.linear.x = 1.0        # moving, so the fix is exercised
        self.odom_pub.publish(m)

    def pub_ref(self):
        p = Path()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'odom'
        for i in range(N + 1):
            ps = PoseStamped()
            ps.header = p.header
            ps.pose.position.x = 20.0 + 0.5 * i
            ps.pose.position.z = 10.0
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        self.path_pub.publish(p)

    def on_traj(self, _msg):
        self.traj_times.append(time.time())

    def on_health(self, msg):
        self.health.append(msg)


def main():
    rclpy.init()
    h = Harness()
    start = time.time()
    # Let the node latch onto the streams before measuring.
    while time.time() - start < 2.0:
        rclpy.spin_once(h, timeout_sec=0.05)
    h.traj_times.clear()

    t_measure_start = time.time()
    while time.time() - t_measure_start < MEASURE_S:
        rclpy.spin_once(h, timeout_sec=0.05)

    n = len(h.traj_times)
    rate = n / MEASURE_S

    print(f"\n--- T1.7 acceptance ---")
    print(f"reference publish rate : {REF_HZ:.1f} Hz")
    print(f"odometry publish rate  : {ODOM_HZ:.1f} Hz")
    print(f"MPC output messages    : {n} in {MEASURE_S:.0f} s -> {rate:.1f} Hz")

    solve_ms = None
    if h.health:
        last = h.health[-1]
        kv = {p.key: p.value for p in last.values}
        solve_ms = float(kv.get('max_solve_ms', 'nan'))
        print(f"health level           : {last.level} ({last.message})")
        print(f"last_solve_ms          : {kv.get('last_solve_ms')}")
        print(f"max_solve_ms           : {kv.get('max_solve_ms')}")
        print(f"solve_count/fail_count : {kv.get('solve_count')}/{kv.get('fail_count')}")
    else:
        print("health                 : NO MESSAGES")

    ok_rate = rate > 3.0 * REF_HZ            # clearly decoupled from the 2 Hz reference
    ok_health = bool(h.health)
    ok_solve = solve_ms is not None and solve_ms < 2.0

    print(f"\nPASS rate decoupled (> {3*REF_HZ:.0f} Hz): {ok_rate}")
    print(f"PASS health published            : {ok_health}")
    print(f"PASS max solve < 2 ms            : {ok_solve}")
    result = ok_rate and ok_health and ok_solve
    print(f"\nT1.7 {'PASS' if result else 'FAIL'}")

    h.destroy_node()
    rclpy.shutdown()
    return 0 if result else 1


if __name__ == '__main__':
    raise SystemExit(main())
