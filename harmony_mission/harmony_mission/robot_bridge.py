#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from idmind_msgs.msg import LockStatus, Voltages, SystemStatus, Bumpers
from idmind_msgs.srv import SetString, Move
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import LaserScan

import json
from collections import deque

TIMEOUT = 5.0

class RobotBridge(Node):
    """ This class will make the connection between the robots ROS2 services and the Mission Handler """
    def __init__(self, node_name='robot_ros2_bridge', verbose=2):
        super().__init__(node_name=node_name)
        self.verbose = verbose
        self.serv_cb_group = ReentrantCallbackGroup()
        self.sub_cb_group = ReentrantCallbackGroup()
        self.pub_cb_group = ReentrantCallbackGroup()
        self.main_cb_group = ReentrantCallbackGroup()

        self.robot_state = {}
        self.robot_state["mission_state"] = None
        self.robot_goals = {}
                
        # Navigation Maps/Locations
        # Load map and map_obs services
        # Load all locations

        # Simple movements
        self.move_cli = self.create_client(Move, "/harmony/move", callback_group=self.serv_cb_group)
        
        # Sensorsboard
        sensorsboard_node = "/harmony/sensors_node"
        self.bumpers = None
        self.voltages = None
        self.system_status = None
        self.bumpers_sub = self.create_subscription(Bumpers, sensorsboard_node+"/bumpers", self.update_bumpers, 10, callback_group=self.sub_cb_group)
        self.system_status_sub = self.create_subscription(SystemStatus, sensorsboard_node+"/system_status", self.update_system_status, 10, callback_group=self.sub_cb_group)
        self.voltages_sub = self.create_subscription(Voltages, sensorsboard_node+"/voltages", self.update_voltages, 10, callback_group=self.sub_cb_group)
        self.undock_cli = self.create_client(Trigger, sensorsboard_node+"/undock", callback_group=self.serv_cb_group)
        self.motor_takeover_cli = self.create_client(SetBool, sensorsboard_node+"/motor_takeover", callback_group=self.serv_cb_group)
        
        # Lasers
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.merged_scan = None
        self.merged_scan_sub = self.create_subscription(LaserScan, "/harmony/merged_scan", self.update_merged_scan, qos_profile=qos_profile, callback_group=self.sub_cb_group)
        
        # Ligths
                
        # Drawers
        top_driver_node = "/harmony/top_driver_node"
        self.robot_state["lock_status"] = {"front": {"locked": None, "ready": None}, "top_tray": {"locked": None, "ready": None}, "bottom_tray": {"locked": None, "ready": None}}
        self.robot_goals["lock_status"] = {"front": {"locked": None, "ready": None}, "top_tray": {"locked": None, "ready": None}, "bottom_tray": {"locked": None, "ready": None}}
        self.lock_status_sub = self.create_subscription(LockStatus, top_driver_node+"/lock_status", self.update_lock_status, 10, callback_group=self.sub_cb_group)
        self.open_lock_future = None
        self.open_lock_cli = []
        self.open_lock_cli.append(self.create_client(Trigger, top_driver_node+"/open_lock1", callback_group=self.serv_cb_group))
        self.open_lock_cli.append(self.create_client(Trigger, top_driver_node+"/open_lock2", callback_group=self.serv_cb_group))
        self.open_lock_cli.append(self.create_client(Trigger, top_driver_node+"/open_lock3", callback_group=self.serv_cb_group))
        
        # Arm
        self.robot_state["arm_status"] = {}
        self.robot_goals["arm_status"] = {}
        self.robot_state["arm_status"]["power"] = None
        self.robot_goals["arm_status"]["power"] = None
        self.arm_power_status_sub = self.create_subscription(Bool, top_driver_node+"/arm_status", self.update_arm_power_status, 10, callback_group=self.sub_cb_group)
        self.arm_power_cli = self.create_client(SetBool, top_driver_node+"/set_arm_power", callback_group=self.serv_cb_group)
        
        arm_controller_node = "/harmony/arm_controller_node"
        self.robot_state["arm_status"]["state"] = None
        self.robot_state["arm_status"]["enable"] = None
        self.robot_goals["arm_status"]["enable"] = None
        self.arm_state_sub = self.create_subscription(String, arm_controller_node+"/arm_state", self.update_arm_state, 10, callback_group=self.sub_cb_group)
        self.arm_enable_cli = self.create_client(SetBool, arm_controller_node+"/arm_enable", callback_group=self.serv_cb_group)
        self.arm_animation_future = None
        self.arm_animation_cli = self.create_client(SetString, arm_controller_node+"/play_animation", callback_group=self.serv_cb_group)

        # HRI 
        self.robot_state["hri_status"] = {}
        self.robot_goals["hri_status"] = {}
        self.hri_preset_pub = self.create_publisher(String, '/harmony/screen/set_hri_preset', 10, callback_group=self.pub_cb_group)
        self.hri_sequence_pub = self.create_publisher(String, '/harmony/screen/set_hri_sequence', 10, callback_group=self.pub_cb_group)
        self.hri_state_sub = self.create_subscription(String, "/harmony/screen/hri_state", self.update_hri_state, 10, callback_group=self.pub_cb_group)
        
        # Mission Web interface
        # This is for interaction with the web interface        
        self.mission_state = {
            "mission_state": "START", # "START", "IDLE", "NAVIGATING", "READY_TO_LOAD_UNLOAD", "LOADING_UNLOADING", "LOADING_COMPLETED", "MISSION_COMPLETED"
            "mission_state_message": "Navigating to X",
            "warning_state": "NONE", # "NONE", "TRAY_ALARM", "BLOCKED"
            "warning_state_message": "Warning: Storage not locked...",
        }
        self.robot_goals["web_interface"] = {}
        self.robot_goals["web_interface"]["state_json"] = None
        self.mission_state_pub = self.create_publisher(String, '/harmony/mission/state_json', 10, callback_group=self.pub_cb_group)
        self.robot_goals["web_interface"]["cmd"] = None
        self.mission_cmd_sub = self.create_subscription(String, '/harmony/mission/cmd', self.update_mission_cmd, 10, callback_group=self.sub_cb_group)

        self.start_pickup_cli = self.create_client(Trigger, "/harmony/mission_controller/start_pickup", callback_group=self.serv_cb_group)
        
        self.enable_alarm_pub = self.create_publisher(Bool, "/harmony/people_detection_node/enable_alarm", 10, callback_group=self.pub_cb_group)
                
        self.clear_wait_task = False
        self.clear_wait_task_sub = self.create_subscription(Bool, "/harmony/mission/clear_wait_task", self.update_clear_wait_task, 10, callback_group=self.sub_cb_group)        

        self.get_logger().info("Node is ready")        

    #################
    #   CALLBACKS   #
    #################
    def update_lock_status(self, msg:LockStatus):
        self.robot_state["lock_status"]["front"] = {"locked": msg.lock1, "ready": msg.ready_lock1}
        self.robot_state["lock_status"]["top_tray"] = {"locked": msg.lock2, "ready": msg.ready_lock2}
        self.robot_state["lock_status"]["bottom_tray"] = {"locked": msg.lock3, "ready": msg.ready_lock3}
        # print("We are receiving lock messages")
    
    def update_merged_scan(self, msg:LaserScan):
        self.merged_scan = msg

    def update_bumpers(self, msg:Bumpers):
        self.bumpers = msg

    def update_voltages(self, msg:Voltages):
        self.voltages = msg

    def update_system_status(self, msg:SystemStatus):
        self.system_status = msg

    def update_arm_power_status(self, msg:Bool):
        self.robot_state["arm_status"]["power"] = msg.data
        # print("We are receiving power messages")
    
    def update_arm_state(self, msg:String):
        self.robot_state["arm_status"]["state"] = msg.data

    def update_hri_state(self, msg:String):
        self.robot_state["hri_status"]["state"] = msg.data

    def update_clear_wait_task(self, msg:Bool):
        self.clear_wait_task = msg.data

    def update_mission_cmd(self, msg:String):
        cmd = msg.data
        # self.robot_state["hri_status"]["state"] = cmd
        self.get_logger().warn("Received command: {}".format(msg))
        # Try to parse commands
        if cmd == "PICKUP":
            self.get_logger().warn("Picking up")
            req = Trigger.Request()
            self.start_pickup_cli.call_async(req)
        if cmd == "OPEN_FRONT":
            self.robot_goals["lock_status"]["front"]["locked"] = False
        elif cmd == "OPEN_TOP_TRAY":
            self.robot_goals["lock_status"]["top_tray"]["locked"] = False
        elif cmd == "OPEN_BOTTOM_TRAY":
            self.robot_goals["lock_status"]["bottom_tray"]["locked"] = False
        elif cmd == "LOADING_COMPLETED":
            self.get_logger().warn("LOADING_COMPLETED")
            self.robot_state["mission_state"] = cmd
        elif cmd == "DELIVERY_READY":
            self.robot_goals["lock_status"]["front"]["locked"] = False
            self.robot_goals["lock_status"]["top_tray"]["locked"] = False
            self.robot_goals["lock_status"]["bottom_tray"]["locked"] = False
            self.robot_state["mission_state"] = cmd
        elif cmd == "DELIVERY_COMPLETED":
            self.robot_state["mission_state"] = cmd
        
        # This is for back compatibility
        elif "PRIORITY" in cmd:
            self.mission_state["priority"] = cmd[9:]
        elif cmd == "START_MISSION":
            req = Trigger.Request()
            self.start_pickup_cli.call_async(req)


    #################
    #   AUXILIARY   #
    #################
    def calc_dt(self, start, stop):
        """ Computes the diffence in seconds between two timestamps """
        dt = stop.sec + stop.nanosec*1e-9 - start.sec - start.nanosec*1e-9
        return dt

    def wait_for_server(self, client, timeout=5.0):
        """ General method to wait for given service """
        # Wait for service
        start = self.get_clock().now().to_msg()
        found = False
        while not found:
            found = client.wait_for_service(timeout_sec=1.0)
            curr_time = self.get_clock().now().to_msg()
            if self.calc_dt(start, curr_time) > timeout:
                self.get_logger().warn("Failed to find {} service...".format(client.srv_name))
                return False
            self.get_logger().warn("Waiting for {} service...".format(client.srv_name))
            return True
    
    def publish_mission_state(self):
        self.mission_state_pub.publish(String(data=json.dumps(self.mission_state)))
        return True

    ###################
    #   GET METHODS   #
    ###################
    def get_clear_wait_task(self):
        return self.clear_wait_task

    def get_lock_status(self):
        return self.robot_state["lock_status"]
    
    def get_lock_goals(self):
        return self.robot_goals["lock_status"]

    def get_arm_power_status(self) ->Bool|None:
        return self.robot_state["arm_status"]["power"]

    def get_arm_power_goals(self) ->Bool|None:
        return self.robot_goals["arm_status"]["power"]

    def get_arm_state(self) -> String|None:
        """ Returns the state reported by the arm_controller: disabled, None or the name of animation playing """
        return self.robot_state["arm_status"]["state"]

    def get_hri_state(self) -> String|None:
        """ Returns the state reported by the HRI system """
        return self.robot_state["hri_status"]["state"]

    def get_mission_state(self) -> String|None:
        """ Returns the mission state reported by the HRI/Web system """
        return self.robot_state["mission_state"]
    
    def get_voltages(self):
        """ Returns battery and cable voltages """
        v = {}
        v["motor_volt"] = self.voltages.motor_battery_voltage
        v["electronic_volt"] = self.voltages.electronic_battery_voltage
        v["cable_volt"] = self.voltages.cable_power_voltage
        return v
    
    def get_bumpers(self):
        """ Returns bumpers state """
        b = {}
        b["front"] = self.bumpers.front
        b["back"] = self.bumpers.back
        b["left"] = self.bumpers.left
        b["right"] = self.bumpers.right
        return b

    def get_laser_scan(self):
        return self.merged_scan
    
    def is_docked(self) -> bool | None:
        """ Return True if the robot is docked (even if not charging) """
        if self.voltages is None:
            return None
        return self.voltages.cable_power_voltage > 10.0

    def is_charging(self) -> bool | None:
        """ Return True if the robot is charging """
        if self.system_status is None:
            return None
        return self.system_status.electronic_battery_charging or self.system_status.motor_battery_charging
    
    def bumpers_active(self):
        """ Return True if any bumper is active """
        if self.bumpers is None:
            return None
        return any([self.bumpers.front, self.bumpers.back, self.bumpers.left, self.bumpers.right])
        
    ###################
    #   SET METHODS   #
    ###################
    def enable_alarm(self, enable=True):
        self.enable_alarm_pub.publish(Bool(data=enable))
        return True

    def clear_lock_goals(self, locks=["front", "top_tray", "bottom_tray"]):
        for lock in locks:
            self.robot_goals["lock_status"][lock]["locked"] = None
    
    def update_mission_state(self, new_mission_state:dict):
        """ Updates the current mission state to be published to the web interface """
        for k, v in new_mission_state.items():
            self.mission_state[k] = v
        self.robot_state["mission_state"] = new_mission_state["mission_state"]
        return True
    
    def motor_takeover(self, enable=True):
        """ Request takeover of the motors to ignore bumpers """
        # Wait for service
        if not self.wait_for_server(self.motor_takeover_cli):
            return False
        
        req = SetBool.Request()
        req.data = enable
        
        future = self.motor_takeover_cli.call_async(req)
        while not future.done():
            print("Waiting for future for motor takeover")
            self.get_clock().sleep_for(Duration(nanoseconds=1e8))
        return future.result().success

    def undock(self):
        """ Request the robot to change relays for undocking """
        # Wait for service
        if not self.wait_for_server(self.undock_cli):
            return False
        
        req = Trigger.Request()
        future = self.undock_cli.call_async(req)
        while not future.done():
            print("Waiting for future for undocking")
            self.get_clock().sleep_for(Duration(nanoseconds=1e8))
        return future.result().success        
        
    def move_forward(self, dist, vel=0.2):
        """ Request the robot to move forward/backward a set distance """
        # Wait for service    
        if not self.wait_for_server(self.move_cli):
            return False
        
        req = Move.Request()
        req.distance = dist
        req.velocity = vel
        req.p = 0.7
        req.a = vel/4
        future = self.move_cli.call_async(req)
        while not future.done():
            print("Waiting for future for moving")
            self.get_clock().sleep_for(Duration(nanoseconds=5e8))
        return future.result().success
        
    def open_lock(self, lock):
        """ Requests opening one of the locks (can be passed by name or number): 
            - front (1)
            - top_tray (2)
            - bottom_tray (3)
        """
        # global executor
        err_msg = "Lock should be in [1, 2, 3] or in ['front', 'top_tray', 'bottom_tray']"
        if type(lock) is str:
            if lock not in ['front', 'top_tray', 'bottom_tray']:
                self.get_logger().error(err_msg)    
                return False
            idx = 0 if lock=="front" else (1 if lock=="top_tray" else 2)
        elif type(lock) is int:
            if lock not in [1, 2, 3]:
                self.get_logger().error(err_msg)    
                return False
            idx = lock-1
        else:
            self.get_logger().error(err_msg)
        
        # Wait for service    
        if not self.wait_for_server(self.open_lock_cli[idx]):
            return False
        
        # Try to open lock
        success = False
        tries = 0
        while not success and tries < 5:
            tries += 1
            req = Trigger.Request()
            future = self.open_lock_cli[idx].call_async(req)
            while not future.done():
                print("Waiting for future")
                self.get_clock().sleep_for(Duration(nanoseconds=1e8))

            # executor.spin_until_future_complete(future=future, timeout_sec=5.0)
            # rclpy.spin_until_future_complete(self, future=future, timeout_sec=5.0)
            r = future.result()
            if r.success:
                self.get_logger().info(r.message)
                success = True
            else:
                self.get_logger().warn(r.message)
        
        return success
    
    def open_lock_not_block(self, lock):
        """ Requests opening one of the locks (can be passed by name or number): 
            - front (1)
            - top_tray (2)
            - bottom_tray (3)
        """
        # global executor
        err_msg = "Lock should be in [1, 2, 3] or in ['front', 'top_tray', 'bottom_tray']"
        if type(lock) is str:
            if lock not in ['front', 'top_tray', 'bottom_tray']:
                self.get_logger().error(err_msg)    
                return False
            idx = 0 if lock=="front" else (1 if lock=="top_tray" else 2)
        elif type(lock) is int:
            if lock not in [1, 2, 3]:
                self.get_logger().error(err_msg)    
                return False
            idx = lock-1
        else:
            self.get_logger().error(err_msg)
        
        if self.open_lock_future is None:
            # Wait for service    
            if not self.wait_for_server(self.open_lock_cli[idx]):
                return False
            req = Trigger.Request()
            self.open_lock_future = self.open_lock_cli[idx].call_async(req)

        if not self.open_lock_future.done():
            return False
        else:
            r = self.open_lock_future.result()
            if r.success:
                self.open_lock_future = None
                return True
            else:
                return False

    def enable_arm(self, enable=True):
        """ Requests to enable/disable Harmony Arm """
        # global executor
        
        # Check if power control is ready
        if self.get_arm_power_status() is None:
            self.get_logger().warn("TopBoard is not ready yet")
            return False
        
        # Change power status if needed
        if self.get_arm_power_status() != enable:
            if not self.wait_for_server(self.arm_power_cli):
                return False
            req = SetBool.Request()
            req.data = enable
            future = self.arm_power_cli.call_async(req)
            # executor.spin_until_future_complete(future=future)
            rclpy.spin_until_future_complete(self, future=future)
            
            r = future.result()
            if r.success:
                self.get_logger().info(r.message)
            else:
                self.get_logger().warn(r.message)
                return False

        # Enable and wait for arm ready (this can take up tp 2min)
        if enable:
            # Check if arm controller is responding
            while self.get_arm_state() is None:
                self.get_logger().info("Waiting for arm controller to start.")
                self.get_clock().sleep_for(Duration(nanoseconds=5e8))
                rclpy.spin_once()
                # executor.spin_once()
            
            if self.get_arm_state()=="disabled":
                if not self.wait_for_server(self.arm_enable_cli):
                    return False
                req = SetBool.Request()
                req.data = enable
                future = self.arm_enable_cli.call_async(req)
                # executor.spin_until_future_complete(future=future)
                rclpy.spin_until_future_complete(self, future=future)
                r = future.result()
                if r.success:
                    self.get_logger().info(r.message)
                else:
                    self.get_logger().warn(r.message)
                    return False
                
                while self.get_arm_state()=="disabled":
                    self.get_logger().info("Waiting for arm state to be idle - {}".format(self.get_arm_state()))
                    self.get_clock().sleep_for(Duration(nanoseconds=5e8))
                    rclpy.spin_once(self)
                    # executor.spin_once()
                
        self.get_logger().info("Arm is {}.".format("enabled" if enable else "disabled"))
        return True
    
    def arm_ready(self):
        """ Plays the sequence of animations that unloads a box from the top tray """
        # global executor
        
        # Check if arm is ready to play
        if self.get_arm_power_status() is None or self.get_arm_state() is None:
            self.get_logger().warn("Harmony xArm is not ready yet")
            return False
        elif not self.get_arm_power_status() or self.get_arm_state()=="disabled":
            self.get_logger().warn("Harmony xArm is not properly powered yet")
            return False
        elif self.get_arm_state() != "None":
            self.get_logger().warn("Harmony xArm is already playing an animation")
            return False
        
        if not self.wait_for_server(self.arm_animation_cli):
            return False

        return True

    def arm_animation(self, animation):
        """ Plays an animation """
        if self.arm_animation_future is None:
            req = SetString.Request()
            req.data = animation
            self.arm_animation_future = self.arm_animation_cli.call_async(req)
        
        if not self.arm_animation_future.done():
            return False
        else:
            r = self.arm_animation_future.result()
            if r.success:
                self.arm_animation_future = None
                return True
            else:
                return False
        

    def arm_unload(self):
        """ Plays the sequence of animations that unloads a box from the top tray """
        # global executor
        
        # Check if arm is ready to play
        if self.get_arm_power_status() is None or self.get_arm_state() is None:
            self.get_logger().warn("Harmony xArm is not ready yet")
            return False
        elif not self.get_arm_power_status() or self.get_arm_state()=="disabled":
            self.get_logger().warn("Harmony xArm is not properly powered yet")
            return False
        elif self.get_arm_state() != "None":
            self.get_logger().warn("Harmony xArm is already playing an animation")
            return False
        
        if not self.wait_for_server(self.arm_animation_cli):
            return False

        for anim in ["open_drawer"]:
            req = SetString.Request()
            req.data = anim
            future = self.arm_animation_cli.call_async(req)
            # executor.spin_until_future_complete(future=future)
            rclpy.spin_until_future_complete(self, future=future)
            r = future.result()
            if r.success:
                self.get_logger().info(r.message)
            else:
                self.get_logger().warn(r.message)
                return False
            
            # Wait for animation to complete
            started = False
            completed = False
            while not completed:
                s = self.get_arm_state()
                if s==anim:
                    started = True
                else:
                    if started:
                        completed = True
        return True

    def set_hri_preset(self, name):
        self.hri_preset_pub.publish(String(data=name))
        return True

    def start_hri_sequence(self, name):
        self.hri_sequence_pub.publish(String(data=name))
        return True

    
# def main(args=None):
#     rclpy.init(args=args)
#     idx = 0
#     global executor
#     unloaded = False
#     try:
#         rob_bridge = RobotBridge()
#         executor = MultiThreadedExecutor(num_threads=6)
#         executor.add_node(rob_bridge)
#         while True:
#             try:
#                 idx += 1
#                 if idx == 50:
#                     rob_bridge.open_lock("front")
#                 if idx == 100:
#                     rob_bridge.open_lock("top_tray")
#                 if idx == 150:
#                     rob_bridge.open_lock("bottom_tray")
#                 # if idx == 100:
#                 #     print(rob_bridge.enable_arm(True))
#                 # if not unloaded:
#                 #     unloaded = rob_bridge.arm_unload()
#                 #     if unloaded:
#                 #         print("Unload is complete")
#                 executor.spin_once()

#             except KeyboardInterrupt:
#                 print('\033[91m'+"Shutting down Robot Bridge"+'\033[0m')
#                 break
            
#         executor.shutdown()
#         # rob_bridge.shutdown()
#         rob_bridge.destroy_node()
    
#     finally:
#         try:
#             rclpy.shutdown()
#         except Exception:
#             pass

# if __name__ == '__main__':
#     main()
