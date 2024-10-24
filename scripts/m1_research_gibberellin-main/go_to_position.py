#!/usr/bin/env python
# coding: utf-8

import rospy
import moveit_commander
import sys
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import tf
import time
import copy

class MoveWithMoveit:
    def __init__(self, move_group_name):
        """move groupを管理するクラス

        Args:
            move_group_name (str) : Moveitのmove groupの名前
        """
        self.move_group_name = move_group_name
        self.__setting_group()

    def subscribe(self):
        rospy.Subscriber("/gibbe/arm/goal", Pose, self.move_route)
        rospy.spin()

    def __setting_group(self):
        """move groupを設定する関数"""
        group = moveit_commander.MoveGroupCommander(self.move_group_name)
        group.set_planner_id("TRRTkConfigDefault")
        group.set_planning_time(2.5)
        group.allow_replanning(True)
        group.set_goal_joint_tolerance(0.07)
        group.set_max_velocity_scaling_factor(0.2)
        group.set_max_acceleration_scaling_factor(0.2)
        self.group = group

    def move_route(self, position):
        """経路生成してアームを動かす関数

        Args:
            position (Pose): 目的地の座標 (Pose)
        """
        group = self.group

        waypoints = []
        listener = tf.TransformListener()
        trans = False
        while not trans and not rospy.is_shutdown():
            try:
                (trans, rot) = listener.lookupTransform(
                    "/base_link", "/J6", rospy.Time(0)
                )
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                continue

        # Get differences between current and target position
        x_diff = position.position.x - trans[0]
        y_diff = position.position.y - trans[1]
        z_diff = position.position.z - trans[2]

        wpose = group.get_current_pose().pose

        # Apply calculated position differences to the current pose
        wpose.position.x += x_diff
        wpose.position.y += y_diff
        wpose.position.z += z_diff

        # Add waypoint based on the new pose
        waypoints.append(copy.deepcopy(wpose))

        # Plan the Cartesian path (allowing for slight deviation)
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)

        # Execute the planned path
        result = group.execute(plan, wait=True)
        print("Execution result:", result)

        # Publish success signal
        pub = rospy.Publisher("/gibbe/arm/quit/goal", Bool, queue_size=1)
        pub.publish(True)

        # Simulate some additional task (like moving up or down) after the motion
        pub_up_down = rospy.Publisher("/gibbe/arm/up_down", Bool, queue_size=10)
        for i in range(5):
            pub_up_down.publish(True)
            time.sleep(1)

        return result

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("arm_go_pos", anonymous=True)
    group_name = "arm"
    ma = MoveWithMoveit(group_name)
    position = Pose()
    position.position.x = 0.5
    position.position.y = 0.2
    position.position.z = 0.7
    ma.move_route(position)
