#!/usr/bin/env python
# coding: utf-8

import rospy
import moveit_commander
import sys
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Bool
import copy

import tf
import time


class MoveWithMoveit:
    def __init__(self, move_group_name):
        """move groupを管理するクラス

        Args:
            move_group_name (str) : Moveitのもmove groupの名前
        """
        self.move_group_name = move_group_name
        self.__setting_group()
        self.data_received = False

    def subscribe(self):
        rospy.Subscriber("/gibbe/arm/back_home", Bool, self.move_route)
        rospy.spin()

    def __setting_group(self):
        """move groupを設定する関数"""
        group = moveit_commander.MoveGroupCommander(self.move_group_name)
        # 経路計画アルゴリズムを指定
        group.set_planner_id("TRRTkConfigDefault")
        # 経路探索にかける時間の上限を指定
        group.set_planning_time(2.5)
        # 経路計画がうまく行かなかったときの再探索を許可
        group.allow_replanning(True)

        # 目標値に対する許容誤差を設定
        group.set_goal_joint_tolerance(0.07)
        # 関節の速度の上限を指定
        group.set_max_velocity_scaling_factor(0.2)
        # group.set_max_velocity_scaling_factor(0.8)
        # 関節の加速度の上限を指定
        group.set_max_acceleration_scaling_factor(0.2)
        self.group = group

    def move_route(self, data):
        """経路生成してアームを動かす関数

        Args:
            goal_joint (np.array) : 目的の関節角度

        Returns:
            result (bool) : 計画･動作が成功したか
        """
        if data and not self.data_received:
            self.data_received = True
            group = self.group

            waypoints = []
            scale = 1
            listener = tf.TransformListener()
            trans = False
            print(data)
            while not trans and not rospy.is_shutdown():
                try:
                    (trans, rot) = listener.lookupTransform(
                        "/base_link", "/J6", rospy.Time(0)
                    )
                    print(trans, rot)

                except (
                    tf.LookupException,
                    tf.ConnectivityException,
                    tf.ExtrapolationException,
                ):
                    continue
            print(trans, rot)

            x_diff = 0.12 - trans[0]
            y_diff = 0 - trans[1]
            z_diff = 0.58 - trans[2]

            wpose = group.get_current_pose().pose
            # wpose.position.z -= scale * 0.1
            # wpose.position.y += scale * 0.2
            # waypoints.append(copy.deepcopy(wpose))
            wpose.position.y += scale * y_diff  # Third move sideways (y)
            wpose.position.z += scale * z_diff  # Third move sideways (y)
            wpose.position.x += scale * x_diff  # Second move forward/backwards in (x)
            # wpose.position.y += scale * 0.1  # Third move sideways (y)
            waypoints.append(copy.deepcopy(wpose))
            
            (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)

            result = group.execute(plan, wait=True)
            print(result)
            return result


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("arm_go_pos", anonymous=True)
    group_name = "arm"
    listener = tf.TransformListener()
    ma = MoveWithMoveit(group_name)
    ma.subscribe()
    # position = [0.5, 0.2, 0.4]
    # ma.move_route(position)
    # rate = rospy.Rate(10.0)
    # while not rospy.is_shutdown():
    #     try:
    #         (trans, rot) = listener.lookupTransform("/base_link", "/J6", rospy.Time(0))
    #     except (
    #         tf.LookupException,
    #         tf.ConnectivityException,
    #         tf.ExtrapolationException,
    #     ):
    #         continue
    #     print(trans, rot)
    #     rate.sleep()
