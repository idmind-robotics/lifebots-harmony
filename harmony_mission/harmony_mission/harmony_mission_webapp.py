#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Trigger
from idmind_msgs.msg import Voltages
from idmind_msgs.srv import SetString

import json
import signal
import threading
from flask import Flask


class MissionClient(Node):
    def __init__(self):
        super().__init__("mission_webapp")
        
        # self.serv_cb_group = ReentrantCallbackGroup()
        # self.sub_cb_group = ReentrantCallbackGroup()
        # self.pub_cb_group = ReentrantCallbackGroup()
        # self.main_cb_group = ReentrantCallbackGroup()

        self.voltages = None
        self.v_sub = self.create_subscription(Voltages, "/harmony/sensors_node/voltages", self.update_voltages, 10)

        self.pickup_future = None
        self.pickup_client = self.create_client(Trigger, "/harmony/mission_controller/start_pickup")
        
        self.add_mission_future = None
        self.add_mission_client = self.create_client(SetString, "/harmony/mission_controller/add_mission")

        self.get_logger().info("Node initiated")

    def update_voltages(self, msg):
        self.voltages = msg
    
    def get_voltages(self) -> Voltages | None:
        return self.voltages
    
    def start_pickup(self):
        if self.pickup_future is None:
            req = Trigger.Request()
            self.pickup_future = self.pickup_client.call_async(req)
            return True
        else:
            return False

    def add_mission(self, params:dict) -> bool:
        if self.add_mission_future is None:
            req = SetString.Request()
            self.add_mission_future = self.add_mission_client.call_async(req)
            return True
        else:
            return False


def ros2_thread(node):
    print('entering ros2 thread')
    rclpy.spin(node)
    print('leaving ros2 thread')

def sigint_handler(signal, frame):
    """
    SIGINT handler

    We have to know when to tell rclpy to shut down, because
    it's in a child thread which would stall the main thread
    shutdown sequence. So we use this handler to call
    rclpy.shutdown() and then call the previously-installed
    SIGINT handler for Flask
    """
    rclpy.shutdown()
    if prev_sigint_handler is not None:
        prev_sigint_handler(signal)

rclpy.init(args=None)
ros2_node = MissionClient()
app = Flask(__name__)
threading.Thread(target=ros2_thread, args=[ros2_node]).start()
prev_sigint_handler = signal.signal(signal.SIGINT, sigint_handler)

@app.route('/voltages')
def get_current_voltages():
    v = ros2_node.get_voltages()
    vd = {}
    vd["electronic_battery_voltage"] = v.electronic_battery_voltage
    vd["motor_battery_voltage"] = v.motor_battery_voltage
    vd["cable_power_voltage"] = v.cable_power_voltage
    return {'voltages': vd}

@app.route('/pickup')
def start_pickup():
    
    if ros2_node.add_mission_client():
        return {"message": "Pickup Started"}
    else:
        return {"message": "Pickup Failed"}
