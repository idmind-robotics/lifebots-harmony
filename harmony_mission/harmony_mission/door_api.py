#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from idmind_msgs.msg import LockStatus, Voltages, SystemStatus, Bumpers
from idmind_msgs.srv import SetString, Move
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import Joy

import json
import numpy as np
from collections import deque

TIMEOUT = 5.0

class DoorAPI(Node):
    """ This class will make the connection between the Mission Handler and the DoorAPI """
    def __init__(self, node_name='door_api', verbose=2):
        super().__init__(node_name=node_name)
        self.verbose = verbose
        self.serv_cb_group = ReentrantCallbackGroup()
        self.sub_cb_group = ReentrantCallbackGroup()
        self.pub_cb_group = ReentrantCallbackGroup()
        self.main_cb_group = ReentrantCallbackGroup()

        self.door_open = False
        self.door_timer = None
        self.joy_sub = self.create_subscription(Joy, "/harmony/joy", self.update_joy, 10 , callback_group=self.sub_cb_group)

        self.get_logger().info("Node is ready")

    #################
    #   CALLBACKS   #
    #################
    def update_joy(self, msg:Joy):
        """ Use the joystick to update the door status """
        # Check for ARROW UP
        # self.get_logger().info("Joy Msg:  {}".format(msg))
        if msg.axes[7]:
            self.door_open = True
            if self.door_timer is not None:
                self.door_timer.destroy()
            self.door_timer = self.create_timer(10.0, self.close_door)
        
    #################
    #   AUXILIARY   #
    #################

    ###################
    #   GET METHODS   #
    ###################
    def get_door_state(self):
        return self.door_open

    ###################
    #   SET METHODS   #
    ###################
    def close_door(self):
        self.door_open = False
        self.door_timer.destroy()
        self.door_timer = None
