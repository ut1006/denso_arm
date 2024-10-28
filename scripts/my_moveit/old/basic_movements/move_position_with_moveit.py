#!/usr/bin/env python
# coding: utf-8

import rospy
import moveit_commander
import sys
from geometry_msgs.msg import PoseStamped, Pose


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
    
    def move_position(self, goal_position):
        """ 経路生成してアームを動かす関数

        Args:
            goal_joint (np.array) : 目的の関節角度

        Returns:
            result (bool) : 計画･動作が成功したか
        """
        group = self.group

        pose_goal = Pose()
        pose_goal.position.x = goal_position[0]
        pose_goal.position.y = goal_position[1]
        pose_goal.position.z = goal_position[2]
        # pose_goal.orientation.w = 1.0
        # 経路のスタート地点を現在地に設定 (なくても良い)
        group.set_start_state_to_current_state()
        # 経路のゴール地点を設定
        group.set_pose_target(pose_goal)
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
    # joints = [0, 0, 0.87, 0, 0, 0]
    # position = [0.3, 0.1, 0.7]
    # position = [0.4, 0.1, 0.8]
    # position = [0.3, 0.1, 0.8]
    position = [0.12, 0.02, 0.58]
    # position = [0.5, 0.23, 0.6]
    ma.move_position(position)
