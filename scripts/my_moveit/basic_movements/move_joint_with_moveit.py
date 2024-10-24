#!/usr/bin/env python
# coding: utf-8

import rospy
import moveit_commander
import sys
from geometry_msgs.msg import PoseStamped


class SetObstacle:
    def __init__(self):
        """ 果実を障害物として設定するクラス

        """
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(1.0)
        self.scene.remove_world_object()

    def remove_sphere_obstacle(self, arm_name):
        """ 果実を障害物として設定する関数

        Args:
            arm_name (str) : アーム名
        """
        all_obstacles = self.scene.get_known_object_names()
        obstacles = [obstacle for obstacle in all_obstacles if arm_name in obstacle]
        for obstacle in obstacles:
            self.scene.remove_world_object(obstacle)

    def __make_pose(self, datum, arm_name):
        """ 障害物の位置からPoseStamped型のデータに変換する関数

        Args:
            datum (list) : 障害物の位置
            arm_name (str) : アーム名

        Returns:
            PoseStamped : 変換後のデータ
        """
        pose = PoseStamped()

        pose.header.frame_id = arm_name + "_base_link"
        pose.pose.position.x = datum[0]
        pose.pose.position.y = datum[1]
        pose.pose.position.z = datum[2]
        pose.pose.orientation.w = 1.0

        return pose

    def add_sphere_obstacle(self, data, arm_name):
        """ 果実を障害物として設定する関数

        Args:
            data (list) : 障害物の位置や番号などのリスト
            arm_name (str) : アーム名
        """
        rospy.loginfo("Set sphere obstacle object ({})".format(arm_name))
        for datum in data:
            pose = self.__make_pose(datum, arm_name)
            self.scene.add_sphere(arm_name + "_target_" + datum[5], pose, datum[3] * 1.2)


class MoveWithMoveit:
    def __init__(self, move_group_name):
        """ move groupを管理するクラス

        Args:
            move_group_name (str) : Moveitのもmove groupの名前
        """
        self.move_group_name = move_group_name
        self.__setting_group()

    def __setting_group(self):
        """ move groupを設定する関数

        """
        group = moveit_commander.MoveGroupCommander(self.move_group_name)
        # 経路計画アルゴリズムを指定
        group.set_planner_id('TRRTkConfigDefault')
        # 経路探索にかける時間の上限を指定
        group.set_planning_time(2.5)
        # 経路計画がうまく行かなかったときの再探索を許可
        group.allow_replanning(True)

        # 目標値に対する許容誤差を設定
        group.set_goal_joint_tolerance(0.07)
        # 関節の速度の上限を指定
        group.set_max_velocity_scaling_factor(1)
        # group.set_max_velocity_scaling_factor(0.8)
        # 関節の加速度の上限を指定
        group.set_max_acceleration_scaling_factor(1)
        self.group = group

    def move(self, goal_joint):
        """ 経路生成してアームを動かす関数

        Args:
            goal_joint (np.array) : 目的の関節角度

        Returns:
            result (bool) : 計画･動作が成功したか
        """
        group = self.group

        # 経路のスタート地点を現在地に設定 (なくても良い)
        group.set_start_state_to_current_state()
        # 経路のゴール地点を設定
        group.set_joint_value_target(goal_joint)
        # 経路計画
        _, motion, _, _ = group.plan()

        # アームを動作
        if len(motion.joint_trajectory.points) > 1:
            result = group.execute(motion)
        else:
            result = False
        return result

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_sender", anonymous=True)
    group_name = "arm"
    ma = MoveWithMoveit(group_name)
    # joints = [0, 0, 0, 0, 0, 0]
    #初期位置の値
    #カメラの都合が悪い方
    # joints = [-2.9385415341733556, -1.4701931529302268, 2.572569981928886, 0.0001989381334289191, -1.062887422748654, 3.0689174375497332]
    # カメラの都合がいい方
    # joints = [-0.0473709328834254, -1.2632056151065432, 2.4254562990911023, 0.006378004494630768, -0.14615208078468495, 3.0564912344298674]
    # 変な動きをしないもの
    joints = [0.0002642521591631124, -0.9422381115788308, 2.362198801732939, -0.04748988803245648, -0.1831435555739088, 3.0049389338389925]
    # joints = [0, 0, 0.87, 0, 0, 0]
    ma.move(joints)
