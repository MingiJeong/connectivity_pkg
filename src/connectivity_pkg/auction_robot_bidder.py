#!/usr/bin/env python

# import of relevant libraries
import rospy # module for ROS APIs
import actionlib
from geometry_msgs.msg import Point32
from std_msgs.msg import Bool 

# custom modules
import aux_function
from auction_robot_super import AuctionRobot, DEFAULT_DESTINATION_ACTION, DEFAULT_REGISTER_SERVICE, DEFAULT_WP_ALLOCATE_INTENTION_TOPIC, DEFAULT_FLUSH_OUT_TOPIC
from connectivity_pkg.srv import ServiceRegistration
from connectivity_pkg.msg import Waypoint_init
from connectivity_pkg.msg import Coordination_DestinationAction, Coordination_DestinationFeedback, Coordination_DestinationResult


# Note: it got a lot reduced as common properties went to Super class
# since auctioneer robot is also bidder at the same time

class AuctionRobotBidder(AuctionRobot):
    def __init__(self, robot_name, robot_total, init_x, init_y, init_z, init_yaw):
        """ Constructor follower robot as inheritance of Superclass"""
        AuctionRobot.__init__(self, robot_name, robot_total, init_x, init_y, init_z, init_yaw)

    def wrap_up_function_bidder(self):
        """
        general wrap up function to be executed in spin of bidder
        """
        # lookup transform
        self._look_up_transform()

        # 1. initial position publish to make TF broadcator can do static TF transform
        self._init_pose_publish()

        # 2. action_server to receive 
        self.goal_action_server()

        # publish rate
        self.rate.sleep()

    def spin(self):
        """
        general spin function for follower robot to loop around 
        Note: not all of them will do things as it is based on finite state machine
        """
        while not rospy.is_shutdown():
            self.wrap_up_function_bidder()