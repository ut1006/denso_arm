#!/usr/bin/env python
# coding: utf-8

from re import A
import rospy
import moveit_commander
import sys
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Bool
import copy
import time

class MoveWithMoveit:
    def __init__(self, move_group_name):
        """ move groupを管理するクラス

        Args:
            move_group_name (str) : Moveitのもmove groupの名前
        """
        self.move_group_name = move_group_name
        self.__setting_group()
        self.data_received = False
    
    def subscribe(self):
        rospy.Subscriber("/gibbe/arm/up_down", Bool, self.up_down)
        rospy.spin()
        
    def up_down(self, data):
        print("up_down")
        if(data and not self.data_received):
            self.data_received = True
            self.move_up()
            time.sleep(2)
            self.move_down()

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
        group.set_max_velocity_scaling_factor(0.2)
        # group.set_max_velocity_scaling_factor(0.8)
        # 関節の加速度の上限を指定
        group.set_max_acceleration_scaling_factor(0.2)
        self.group = group

    def move_up(self):
        """ 経路生成してアームを動かす関数

        Args:
            goal_joint (np.array) : 目的の関節角度

        Returns:
            result (bool) : 計画･動作が成功したか
        """
        group = self.group

        waypoints = []
        scale = 1

        wpose = group.get_current_pose().pose
        # wpose.position.z -= scale * 0.1
        # wpose.position.y += scale * 0.2
        # waypoints.append(copy.deepcopy(wpose))

        wpose.position.z += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        # wpose.position.y -= scale * 0.1  # Third move sideways (y)
        # waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = group.compute_cartesian_path(
            waypoints, 0.01, 0.0
        )

        result = group.execute(plan, wait=True)
        return result

    def move_down(self):
        """ 経路生成してアームを動かす関数

        Args:
            goal_joint (np.array) : 目的の関節角度

        Returns:
            result (bool) : 計画･動作が成功したか
        """
        group = self.group

        waypoints = []
        scale = 1

        wpose = group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = group.compute_cartesian_path(
            waypoints, 0.01, 0.0
        )

        result = group.execute(plan, wait=True)
        pub_go_back = rospy.Publisher("/gibbe/arm/back_home", Bool, queue_size=1)
        for i in range(5):
            pub_go_back.publish(True)
            time.sleep(1)
        return result

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("arm_up_down", anonymous=True)
    group_name = "arm"
    ma = MoveWithMoveit(group_name)
    # joints = [0, 0, 0.87, 0, 0, 0]
    # position = [0.3, 0.1, 0.7]
    # position = [0.4, 0.1, 0.8]
    # position = [0.3, 0.1, 0.8]
    # position = [0.5, 0.23, 0.6]
    ma.subscribe()
    # ma.move_up()
    # time.sleep(3)
    # ma.move_down()
