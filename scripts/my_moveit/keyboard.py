#!/usr/bin/env python
from __future__ import print_function
from six.moves import input
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
try:
    from math import pi, tau, dist, fabs, cos
except:
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class MoveGroupPythonInterfaceTutorial(object):
    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

    def print_current_pose(self):
        current_pose = self.move_group.get_current_pose().pose
        print("Current Position: x={:.3f}, y={:.3f}, z={:.3f}".format(
            current_pose.position.x, 
            current_pose.position.y, 
            current_pose.position.z
        ))
        print("Current Orientation: qx={:.3f}, qy={:.3f}, qz={:.3f}, qw={:.3f}".format(
            current_pose.orientation.x, 
            current_pose.orientation.y, 
            current_pose.orientation.z, 
            current_pose.orientation.w
        ))

    def move_relative(self, dx, dy, dz):
        # 現在のポーズを取得し、相対的に移動する新しいポーズを設定
        current_pose = self.move_group.get_current_pose().pose
        new_pose = geometry_msgs.msg.Pose()
        new_pose.position.x = current_pose.position.x + dx
        new_pose.position.y = current_pose.position.y + dy
        new_pose.position.z = current_pose.position.z + dz
        new_pose.orientation = current_pose.orientation  # 向きは変更しない

        self.move_group.set_pose_target(new_pose)

        try:
            success = self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            if not success:
                rospy.logerr("Failed to move to the specified relative position.")
            return success
        except Exception as e:
            rospy.logerr("An error occurred: %s", e)
            return False


def main():
    tutorial = MoveGroupPythonInterfaceTutorial()

    while not rospy.is_shutdown():
        tutorial.print_current_pose()

        try:
            print("Enter relative movement distance (in cm):")
            dx = float(input("dx (cm): ")) / 100.0
            dy = float(input("dy (cm): ")) / 100.0
            dz = float(input("dz (cm): ")) / 100.0

            if tutorial.move_relative(dx, dy, dz):
                print("Relative move successful!")
            else:
                print("Failed to complete the relative move.")

        except ValueError:
            print("Invalid input. Please enter numeric values.")
        except rospy.ROSInterruptException:
            break
        except KeyboardInterrupt:
            break


if __name__ == "__main__":
    main()
