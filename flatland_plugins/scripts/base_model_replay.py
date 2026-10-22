#!/usr/bin/env python3
"""ROS 1 node (python3, rospy; runs in the flatland process environment): replay recorded drives open loop.

For each drive file in a directory, in time order: publish the recorded commands on `cmd_vel` as simulated
time reaches their stamps, record the ground-truth pose until the drive's end plus a settle time, and write
`<out>/<name>.json` as `validate.load_track` reads it. Drives run one after the other in one flatland
instance; the robot is at rest between drives (a zero command is published first and the settle time lets
the shaped state reach exactly zero) and is moved back to the origin with the `move_model` service. Poses are in
the world frame; the host side normalises them to the start pose of each drive.

Time comes from `/clock` only, so the node follows fast simulation unchanged. The commands reach flatland
through a topic, which adds the hop of a replay path; `validate plugin-vs-offline --hop` accounts for it.

    flatland_replay.py DRIVES_DIR OUT_DIR [--cmd-topic cmd_vel] [--pose-topic odometry/ground_truth]
"""
import argparse
import json
import sys
from pathlib import Path

import rospy
from flatland_msgs.srv import MoveModel
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry

SETTLE_S = 3.0
GAP_S = 3.0


def load_drive(path):
    raw = json.loads(Path(path).read_text())
    t0 = raw["t0"]
    ev = [(float(t) - t0, float(v), float(w)) for t, v, w in raw["cmd"]]
    if ev[-1][1] != 0.0 or ev[-1][2] != 0.0:
        ev.append((ev[-1][0], 0.0, 0.0))
    length = raw["t1"] - t0
    if raw.get("score_end_s") is not None:
        length = min(length, raw["score_end_s"])
    return raw["name"], raw["t0"], ev, length


def yaw_of(q):
    import math
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Replay:
    def __init__(self, cmd_topic, pose_topic, robot):
        rospy.wait_for_service("move_model")
        self.move = rospy.ServiceProxy("move_model", MoveModel)
        self.robot = robot
        self.pub = rospy.Publisher(cmd_topic, Twist, queue_size=1, latch=False)
        self.sub = rospy.Subscriber(pose_topic, Odometry, self.on_pose, queue_size=1000)
        self.samples = []
        self.recording = False

    def on_pose(self, m):
        if self.recording:
            p = m.pose.pose
            self.samples.append((m.header.stamp.to_sec(), p.position.x, p.position.y, yaw_of(p.orientation)))

    def sim_wait(self, until):
        while not rospy.is_shutdown() and rospy.get_time() < until:
            rospy.sleep(0.001)

    def command(self, v, w):
        m = Twist()
        m.linear.x, m.angular.z = v, w
        self.pub.publish(m)

    def run_drive(self, name, ev, length, out_dir):
        self.command(0.0, 0.0)
        self.move(self.robot, Pose2D(0.0, 0.0, 0.0))   # every drive starts at the origin: float precision does not depend on the order
        self.sim_wait(rospy.get_time() + 0.2)
        self.samples = []
        start = rospy.get_time()
        self.recording = True
        i = 0
        while not rospy.is_shutdown():
            now = rospy.get_time() - start
            while i < len(ev) and ev[i][0] <= now:
                self.command(ev[i][1], ev[i][2])
                i += 1
            if now >= length + SETTLE_S and i >= len(ev):
                break
            rospy.sleep(0.001)
        self.recording = False
        self.command(0.0, 0.0)
        s = self.samples
        Path(out_dir, f"{name}.json").write_text(json.dumps({
            "name": name, "t": [r[0] - start for r in s], "x": [r[1] for r in s], "y": [r[2] for r in s],
            "yaw": [r[3] for r in s]}))
        self.sim_wait(rospy.get_time() + GAP_S)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("drives_dir")
    ap.add_argument("out_dir")
    ap.add_argument("--cmd-topic", default="cmd_vel")
    ap.add_argument("--pose-topic", default="odometry/ground_truth")
    ap.add_argument("--robot", default="robot", help="model name, for the move_model service")
    args = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node("flatland_replay")
    Path(args.out_dir).mkdir(parents=True, exist_ok=True)
    while rospy.get_time() == 0.0 and not rospy.is_shutdown():
        rospy.sleep(0.05)   # wait for /clock
    drives = sorted((load_drive(p) for p in Path(args.drives_dir).glob("*.json") if p.name != "index.json"),
                    key=lambda d: d[1])
    node = Replay(args.cmd_topic, args.pose_topic, args.robot)
    rospy.sleep(1.0)
    for name, _t0, ev, length in drives:
        node.run_drive(name, ev, length, args.out_dir)
        rospy.loginfo("replayed %s", name)


if __name__ == "__main__":
    sys.exit(main())
