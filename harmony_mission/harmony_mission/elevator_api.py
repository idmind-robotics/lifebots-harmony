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

class ElevatorAPI(Node):
    """ This class will make the connection between the Mission Handler and the ElevatorAPI """
    def __init__(self, node_name='elevator_api', verbose=2):
        super().__init__(node_name=node_name)
        self.verbose = verbose
        self.serv_cb_group = ReentrantCallbackGroup()
        self.sub_cb_group = ReentrantCallbackGroup()
        self.pub_cb_group = ReentrantCallbackGroup()
        self.main_cb_group = ReentrantCallbackGroup()

        # self.update_elevator_timer = self.create_timer(timer_period_sec=2, callback=self.update_elevator, callback_group=self.serv_cb_group)
        self.current_floor = 0
        self.goal_floor = 0
        self.door_open = False
        self.elevator_timer = None
        self.elevator_finished = False
        self.joy_sub = self.create_subscription(Joy, "/harmony/joy", self.update_joy, 10 , callback_group=self.sub_cb_group)

        self.get_logger().info("Node is ready")

    #################
    #   CALLBACKS   #
    #################
    def update_joy(self, msg:Joy):
        """ Use the joystick to update the door status """
        # If we are in the correct floor, allow doors to open and close
        if msg.axes[7]:
            self.get_logger().warn("Elevator is finished")
            self.elevator_finished = True
            if self.elevator_timer is not None:
                self.elevator_timer.destroy()
            self.elevator_timer = self.create_timer(10.0, self.close_door)

    #################
    #   AUXILIARY   #
    #################

    ###################
    #   GET METHODS   #
    ###################
    def get_current_floor(self):
        return self.current_floor
    
    def get_door_open(self):
        return self.door_open

    def get_elevator_moving(self):
        return self.goal_floor != self.current_floor

    def is_elevator_finished(self):
        return self.elevator_finished

    ###################
    #   SET METHODS   #
    ###################
    def close_door(self):
        self.elevator_finished = False
        self.elevator_timer.destroy()

    def call_elevator(self, floor:int):
        self.get_logger().info("Called elevator to floor {}".format(floor))
        self.elevator_finished = False
        return True

    def press_floor(self, floor:str):
        self.get_logger().info("Pressed button to floor {}".format(floor))
        self.goal_floor = floor
        self.door_open = False
        return True