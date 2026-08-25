#!/usr/bin/env python3
"""T1.7 safety check: the MPC stops solving on a stale reference.

Publishes odom + reference normally, then STOPS the reference and verifies the
node (a) stops emitting setpoints and (b) reports ERROR-level health, rather than
continuing to feed the controller a setpoint derived from a dead planner.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from custom_trajectory_msgs.msg import StateTrajectory
from diagnostic_msgs.msg import DiagnosticStatus
import time

N = 10
REF_TIMEOUT = 0.5


class Harness(Node):
    def __init__(self):
        super().__init__('t17_stale_harness')
        self.odom_pub = self.create_publisher(Odometry, 'mpc/in/odom', qos_profile_sensor_data)
        self.path_pub = self.create_publisher(Path, 'mpc/in/ref_traj_path', qos_profile_sensor_data)
        self.create_subscription(StateTrajectory, 'mpc/out/trajectory', self.on_traj, 10)
        self.create_subscription(DiagnosticStatus, 'mpc/out/health', self.on_health, 10)
        self.traj_times = []
        self.health = []
        self.publish_ref = True
        self.create_timer(1.0 / 30.0, self.pub_odom)
        self.create_timer(1.0 / 10.0, self.pub_ref)

    def pub_odom(self):
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_link'
        m.pose.pose.position.z = 10.0
        m.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(m)

    def pub_ref(self):
        if not self.publish_ref:
            return
        p = Path()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'odom'
        for i in range(N + 1):
            ps = PoseStamped()
            ps.header = p.header
            ps.pose.position.x = 20.0
            ps.pose.position.z = 10.0
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        self.path_pub.publish(p)

    def on_traj(self, _m):
        self.traj_times.append(time.time())

    def on_health(self, m):
        self.health.append((time.time(), m))


def spin_for(node, seconds):
    t = time.time()
    while time.time() - t < seconds:
        rclpy.spin_once(node, timeout_sec=0.02)


def main():
    rclpy.init()
    h = Harness()

    spin_for(h, 3.0)                     # healthy phase
    healthy = len(h.traj_times)
    print(f"healthy phase: {healthy} setpoints in 3 s")

    h.publish_ref = False                # reference dies
    spin_for(h, REF_TIMEOUT + 0.5)       # let the timeout elapse
    h.traj_times.clear()
    h.health.clear()

    spin_for(h, 2.0)                     # measure the stale phase
    stale_setpoints = len(h.traj_times)
    levels = [int.from_bytes(m.level, 'big') if isinstance(m.level, bytes) else int(m.level)
              for _, m in h.health]
    msgs = {m.message for _, m in h.health}

    print(f"stale phase  : {stale_setpoints} setpoints in 2 s (expect 0)")
    print(f"health levels: {sorted(set(levels))} messages={msgs}")

    ok_healthy = healthy > 0
    ok_stopped = stale_setpoints == 0
    ok_error = bool(levels) and all(l == 2 for l in levels)   # 2 == ERROR

    print(f"\nPASS solved while fresh      : {ok_healthy}")
    print(f"PASS stopped when stale      : {ok_stopped}")
    print(f"PASS reported ERROR health   : {ok_error}")
    result = ok_healthy and ok_stopped and ok_error
    print(f"\nT1.7 staleness gate {'PASS' if result else 'FAIL'}")

    h.destroy_node()
    rclpy.shutdown()
    return 0 if result else 1


if __name__ == '__main__':
    raise SystemExit(main())
