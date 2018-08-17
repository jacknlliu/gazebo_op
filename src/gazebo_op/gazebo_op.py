#!/usr/bin/env python

import os
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelState, SetModelConfiguration, DeleteModel, \
    SpawnModel, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose

from sensor_msgs.msg import JointState
from math import pi
from copy import deepcopy
import time
from threading import Timer
# import transforms3d as tf3d

# possible packages
# from control_msgs.msg import FollowJointTrajectoryAction, \
#     FollowJointTrajectoryGoal
# from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
# from actionlib import SimpleActionClient
# from trajectory_msgs.msg import JointTrajectoryPoint
# from tf.transformations import quaternion_from_euler
# from tf_conversions import posemath, toMsg
# import PyKDL
# from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
# from moveit_msgs.srv import GetPlanningScene
# from moveit_commander import MoveGroupCommander


class GazeboOp(object):
    """
    Communication and operation with gazebo
    """

    def __init__(self):
        """
        This constructor initialises the different necessary connections to the topics and services
        and resets the world to start in a good position.

        gazebo ros services:
        /gazebo/apply_body_wrench
        /gazebo/apply_joint_effort
        /gazebo/clear_body_wrenches
        /gazebo/clear_joint_forces
        /gazebo/delete_light
        /gazebo/delete_model
        /gazebo/get_joint_properties
        /gazebo/get_light_properties
        /gazebo/get_link_properties
        /gazebo/get_link_state
        /gazebo/get_loggers
        /gazebo/get_model_properties
        /gazebo/get_model_state
        /gazebo/get_physics_properties
        /gazebo/get_world_properties
        /gazebo/pause_physics
        /gazebo/reset_simulation
        /gazebo/reset_world
        /gazebo/set_joint_properties
        /gazebo/set_light_properties
        /gazebo/set_link_properties
        /gazebo/set_link_state
        /gazebo/set_logger_level
        /gazebo/set_model_configuration
        /gazebo/set_model_state
        /gazebo/set_parameters
        /gazebo/set_physics_properties
        /gazebo/spawn_sdf_model
        /gazebo/spawn_urdf_model
        /gazebo/unpause_physics
        """
        rospy.init_node("gazebo_comm")

        self.__current_model_name = None
        self.__default_path_to_models = os.path.expanduser("~/.gazebo/models/")

        rospy.wait_for_service("/gazebo/get_model_state", 10.0)
        rospy.wait_for_service("/gazebo/reset_world", 10.0)
        self.__reset_world = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        self.__get_pose_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        rospy.wait_for_service("/gazebo/pause_physics")
        self.__pause_physics = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        rospy.wait_for_service("/gazebo/unpause_physics")
        self.__unpause_physics = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        # rospy.wait_for_service("/controller_manager/switch_controller")
        # self.__switch_ctrl = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
        rospy.wait_for_service("/gazebo/set_model_configuration")
        self.__set_model = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)

        rospy.wait_for_service("/gazebo/set_model_state")
        self.__set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        rospy.wait_for_service("/gazebo/delete_model")
        self.__delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        self.__spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

        self.reset_world()


    def reset_world(self):
        # pause physics
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.__pause_physics()
        except (rospy.ServiceException) as exc:
            print("/gazebo/pause_physics service call failed:" + str(exc))

        # set robot model configuration

        self.__reset_world.call()

        self.__unpause_physics.call()

        time.sleep(0.1)


    def pause_physics(self):
        # pause physics
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.__pause_physics()
        except (rospy.ServiceException) as ex:
            print("/gazebo/pause_physics service call failed:" + str(ex))

    def unpause_physics(self):
        self.__unpause_physics.call()


    def set_model_pose(self, model_name, position, orientation):
        state = ModelState()
        state.model_name = model_name
        state.reference_frame = "world"
        [state.pose.position.x, state.pose.position.y, state.pose.position.z] = position
        [state.pose.orientation.w, state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z] = orientation
        [state.twist.linear.x, state.twist.linear.y, state.twist.linear.z] = [0.0] *3
        [state.twist.angular.x, state.twist.angular.y, state.twist.angular.z] = [0.0] *3
        self.__set_model_state(state)

    def generate_box_model(self, model_size):
        pass

    def generate_sphere_model(self, model_size):
        pass
    
    def generate_cylinder_model(self, model_size):
        pass

    def spawn_model(self, model_name, robot_ns=None, initial_pose=None, is_basic_model=False):
        try:
            sdf = None
            init_pose = Pose()

            if  initial_pose is not None:
                [init_pose.position.x, init_pose.position.y, init_pose.position.z,
                init_pose.orientation.w, init_pose.orientation.x, init_pose.orientation.y,
                init_pose.orientation.z] = initial_pose
                print("the init pose is: {}".format(init_pose))
            else:
                init_pose.position.x = 0.15
                init_pose.position.z = 0.82

            if not is_basic_model:
                with open(self.__default_path_to_models + model_name + "/model.sdf", "r") as model:
                    sdf = model.read()
            else:
                sdf = generate_box_model([0.1, 0.2, 0.3])
            print("we set init pose is: {}".format(init_pose))
            res = self.__spawn_model(model_name, sdf, robot_ns, init_pose, "world")
            rospy.loginfo("RES: " + str(res))
            self.__current_model_name = model_name
        except:
            rospy.logwarn("Failed to delete: " + self.__current_model_name)

    def delete_model(self, model_name):
        try:
            self.__delete_model(model_name)
        except:
            rospy.logwarn("Failed to delete: " + model_name)