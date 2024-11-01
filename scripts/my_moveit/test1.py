#!/usr/bin/env python

'''位置指令を逐次発出。sleepなしで司令して大丈夫だった。デカルト軌道はset_max_velができず不安定'''
# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):

    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )


        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self):
        move_group = self.move_group

        move_group.set_max_velocity_scaling_factor(0.5)  # 50% of max velocity
        move_group.set_max_acceleration_scaling_factor(0.5)  # 50% of max accellalation

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = 0  

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def print_current_pose(self):
        current_pose = self.move_group.get_current_pose().pose
        
        rospy.loginfo("Current Pose:")
        rospy.loginfo("Position: [x: %.3f, y: %.3f, z: %.3f]", 
                      current_pose.position.x, 
                      current_pose.position.y, 
                      current_pose.position.z)
        rospy.loginfo("Orientation: [x: %.3f, y: %.3f, z: %.3f, w: %.3f]", 
                      current_pose.orientation.x, 
                      current_pose.orientation.y, 
                      current_pose.orientation.z, 
                      current_pose.orientation.w)


    def go_to_pose_goal(self):
        move_group = self.move_group

        move_group.set_max_velocity_scaling_factor(0.5)  # 50% of max velocity
        move_group.set_max_acceleration_scaling_factor(0.5)  # 50% of max acceleration

        current_pose = move_group.get_current_pose().pose
        pose_goal = copy.deepcopy(current_pose)

        print("Control the robot with the following keys:")
        print("  w/s - Move along x-axis")
        print("  a/d - Move along y-axis")
        print("  q/e - Move along z-axis")
        print("  i/k - Rotate around x-axis")
        print("  j/l - Rotate around y-axis")
        print("  u/o - Rotate around z-axis")
        print("Press 'Enter' to execute movement or 'Ctrl+C' to exit.")

        step_size = 0.01  # Set the step size for translation
        rotation_step = 0.1  # Set the step size for rotation in quaternion

        while True:
            key = input("Enter direction key: ").strip().lower()

            # Translation controls
            if key == 'w':
                pose_goal.position.x += step_size
            elif key == 's':
                pose_goal.position.x -= step_size
            elif key == 'a':
                pose_goal.position.y += step_size
            elif key == 'd':
                pose_goal.position.y -= step_size
            elif key == 'q':
                pose_goal.position.z += step_size
            elif key == 'e':
                pose_goal.position.z -= step_size

            # Orientation controls (adjust quaternion values directly)
            elif key == 'i':
                pose_goal.orientation.x += rotation_step
            elif key == 'k':
                pose_goal.orientation.x -= rotation_step
            elif key == 'j':
                pose_goal.orientation.y += rotation_step
            elif key == 'l':
                pose_goal.orientation.y -= rotation_step
            elif key == 'u':
                pose_goal.orientation.z += rotation_step
            elif key == 'o':
                pose_goal.orientation.z -= rotation_step

            # Exit the loop if 'Enter' is pressed
            elif key == '':
                break
            else:
                print("Invalid key. Use the keys specified above.")
                continue

            move_group.set_pose_target(pose_goal)
            success = move_group.go(wait=True)
            move_group.stop()
            move_group.clear_pose_targets()

            if not success:
                print("Movement failed. Try a different key.")

        # Return to check if the final pose is within tolerance
        return all_close(pose_goal, move_group.get_current_pose().pose, 0.01)



    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene


        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL



def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        input(
            "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        )
        tutorial = MoveGroupPythonInterfaceTutorial()

        tutorial.print_current_pose()  
        # input(
        #     "============ Press `Enter` to execute a movement using a joint state goal ..."
        # )
        # tutorial.go_to_joint_state()

        input("============ Press `Enter` to execute a movement using a pose goal ...")
        tutorial.go_to_pose_goal()



    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()


