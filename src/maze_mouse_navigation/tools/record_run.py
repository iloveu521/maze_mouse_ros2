#!/usr/bin/env python3
"""Record one Gazebo run for controller tuning; navigation stays in C++20."""

import csv
import json
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


def main():
    if len(sys.argv) not in (2, 3):
        raise SystemExit("usage: record_run.py OUTPUT_PREFIX [MAX_SECONDS]")
    prefix = Path(sys.argv[1])
    max_seconds = float(sys.argv[2]) if len(sys.argv) == 3 else 360.0
    if max_seconds <= 0:
        raise SystemExit("MAX_SECONDS must be positive")
    prefix.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("maze_tuning_recorder")
    odom_file = prefix.with_suffix(".odom.csv").open("w")
    command_file = prefix.with_suffix(".commands.csv").open("w")
    status_file = prefix.with_suffix(".status.jsonl").open("w")
    odom_writer = csv.writer(odom_file)
    command_writer = csv.writer(command_file)
    odom_writer.writerow(["stamp_s", "x", "y", "linear", "angular"])
    command_writer.writerow(["stamp_s", "linear", "angular"])
    latest = None
    terminal_at = None

    def odom(message):
        stamp = message.header.stamp
        odom_writer.writerow([
            stamp.sec + stamp.nanosec / 1e9,
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.twist.twist.linear.x,
            message.twist.twist.angular.z,
        ])

    def command(message):
        command_writer.writerow([
            node.get_clock().now().nanoseconds / 1e9,
            message.linear.x,
            message.angular.z,
        ])

    def status(message):
        nonlocal latest, terminal_at
        status_file.write(message.data + "\n")
        latest = json.loads(message.data)
        if latest["state"] in ("COMPLETE", "FAULT", "NO_FRONTIER") and terminal_at is None:
            terminal_at = time.monotonic()

    node.create_subscription(Odometry, "/odom", odom, qos_profile_sensor_data)
    node.create_subscription(Twist, "/cmd_vel", command, 10)
    node.create_subscription(String, "/maze_mouse/status", status, 10)
    start = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - start < max_seconds:
            rclpy.spin_once(node, timeout_sec=0.2)
            if terminal_at is not None and time.monotonic() - terminal_at > 0.7:
                break
    finally:
        odom_file.close()
        command_file.close()
        status_file.close()
        if latest is not None:
            prefix.with_suffix(".result.json").write_text(
                json.dumps(latest, indent=2) + "\n"
            )
            print(json.dumps(latest), flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
