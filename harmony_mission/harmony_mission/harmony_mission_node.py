#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Trigger
from idmind_msgs.srv import SetMission
from std_srvs.srv._trigger import Trigger_Response
from sensor_msgs.msg import Joy

from .missions import *
from .robot_bridge import RobotBridge
from .map_manager import MapManager
from .elevator_api import ElevatorAPI
from .door_api import DoorAPI

import json

class MissionController(Node):
    def __init__(self, robot_bridge:RobotBridge, map_manager:MapManager, elevator_api:ElevatorAPI, door_api: DoorAPI, node_name='mission_controller_node'):
        super().__init__(node_name=node_name)
        
        self.serv_cb_group = ReentrantCallbackGroup()
        self.pub_cb_group = ReentrantCallbackGroup()
        self.main_cb_group = ReentrantCallbackGroup()

        node_prefix = self.get_name()+"/"
        self.ready = False
        self.ready_srv = self.create_service(Trigger, node_prefix+"ready", self.report_ready, callback_group=self.serv_cb_group)

        self.control_freq = 10.0
        self.missions = MissionQueue()
        self.cancel_mission_flag = False
        self.docked = False
        # self.start_mission_srv = self.create_service(Trigger, node_prefix+"start_testing", self.handle_start_testing, callback_group=self.serv_cb_group)
        # self.start_docking_srv = self.create_service(Trigger, node_prefix+"start_docking", self.handle_start_docking, callback_group=self.serv_cb_group)
        # self.start_undocking_srv = self.create_service(Trigger, node_prefix+"start_undocking", self.handle_start_undocking, callback_group=self.serv_cb_group)
        self.start_pickup_srv = self.create_service(Trigger, node_prefix+"start_pickup", self.handle_start_pickup, callback_group=self.serv_cb_group)
        # self.start_delivery_srv = self.create_service(Trigger, node_prefix+"start_delivery", self.handle_start_delivery, callback_group=self.serv_cb_group)
        self.start_vodafone_srv = self.create_service(Trigger, node_prefix+"start_vodafone", self.handle_start_vodafone, callback_group=self.serv_cb_group)
        self.start_vodafone_arm_srv = self.create_service(Trigger, node_prefix+"start_vodafone_arm", self.handle_start_vodafone_arm, callback_group=self.serv_cb_group)

        
        self.add_mission_srv = self.create_service(SetMission, node_prefix+"add_mission", self.handle_add_mission, callback_group=self.serv_cb_group)
        self.cancel_mission_srv = self.create_service(Trigger, node_prefix+"cancel_mission", self.handle_cancel_mission, callback_group=self.serv_cb_group)

        self.joy = None
        self.joy_sub = self.create_subscription(Joy, "/harmony/joy", self.update_joy, 10, callback_group=self.pub_cb_group)
        
        self.navigator = BasicNavigator()
        self.robot_bridge = robot_bridge
        self.map_manager = map_manager
        self.elevator = elevator_api
        self.door_api = door_api
        
        self.ready = True
        self.in_loop = False
        
        self.relocate_to_dock = False
        
        self.main_timer = self.create_timer(1.0/self.control_freq, self.start, callback_group=self.main_cb_group)
        self.get_logger().info("Node initiated")

    ###################
    #    CALLBACKS    #
    ###################
    def report_ready(self, _req, response:Trigger.Response) -> Trigger_Response:
        """ Simple Service callback to show node is ready """
        self.get_logger().info("Replying to 'ready' request")
        response.success = self.ready
        response.message = self.get_name()+" is " + ("ready" if self.ready else "not ready")
        return response

    def handle_add_mission(self, req:SetMission.Request, res:SetMission.Response) -> SetMission.Response:  
        """ Checks request and adds a new mission to the list """
        name = req.name
        type = req.type
        map_name = req.map_name
        location = req.location
        
        params = {}
        if len(req.params) > 0:
            try:
                params = json.loads(req.params)
            except json.decoder.JSONDecodeError:
                msg = "Exception decoding the params"
                self.get_logger().error(msg)
                res.success = False
                res.message = msg
                return res

        if type == "Dock":
            mission = DockMission(name=name, destination=location, map_name=map_name, 
                                  navigator=self.navigator, robot=self.robot_bridge, 
                                  map_manager=self.map_manager)
            self.load_mission(mission)
        elif type == "LoadCargo":
            mission = LoadCargoMission(name=name, location=location, map_name=map_name, 
                                       navigator=self.navigator, robot=self.robot_bridge, 
                                       map_manager=self.map_manager, elevator_api=self.elevator)
            self.load_mission(mission)
        elif type == "DeliverCargo":
            mission = DeliverCargoMission(name=name, location=location, map_name=map_name, 
                                          navigator=self.navigator, robot=self.robot_bridge, 
                                          map_manager=self.map_manager, elevator_api=self.elevator)
            self.load_mission(mission)
        elif type == "CUFPickup":
            mission = CUFPickup(name=name, location=location, map_name=map_name, 
                                navigator=self.navigator, robot=self.robot_bridge, 
                                map_manager=self.map_manager, elevator_api=self.elevator, door_api=self.door_api)
            self.load_mission(mission)
        elif type == "CUFDelivery":
            mission = CUFDelivery(name=name, location=location, map_name=map_name, 
                                navigator=self.navigator, robot=self.robot_bridge, 
                                map_manager=self.map_manager, elevator_api=self.elevator, door_api=self.door_api)
            self.load_mission(mission)
        elif type == "CUFDelivery2":
            mission = CUFDelivery2(name=name, location=location, map_name=map_name, 
                                navigator=self.navigator, robot=self.robot_bridge, 
                                map_manager=self.map_manager, elevator_api=self.elevator, door_api=self.door_api)
            self.load_mission(mission)
        elif type == "CUFDelivery3":
            mission = CUFDelivery3(name=name, location=location, map_name=map_name, 
                                navigator=self.navigator, robot=self.robot_bridge, 
                                map_manager=self.map_manager, elevator_api=self.elevator, door_api=self.door_api)
            self.load_mission(mission)
        elif type == "CUFArmDelivery":
            mission = CUFArmDelivery(name=name, location=location, map_name=map_name, 
                                navigator=self.navigator, robot=self.robot_bridge, 
                                map_manager=self.map_manager, elevator_api=self.elevator, door_api=self.door_api)
            self.load_mission(mission)
        else:
            self.get_logger().error("Unknown Mission Type.")
            res.success = False
            res.message = "Unknown Mission Type. Choose from 'Dock', 'LoadCargo' or 'DeliverCargo'"
            return res
            
        res.success = True
        res.message = "Mission added to plan"
        return res

    def handle_cancel_mission(self, _req, res):
        """ Cancels all missions """
        self.cancel_mission_flag = True
        res.success = True
        res.message = "Cancel mission requested"
        return res

    def handle_start_vodafone(self, _req, res):
        """ Starts the Vodafaone mission """
        new_mission = VodafoneMission("vodafone_mission", navigator=self.navigator, robot=self.robot_bridge, map_manager=self.map_manager)
        self.load_mission(new_mission)
        res.success = True
        res.message = "Starting Vodafone Mission"
        return res

    def handle_start_vodafone_arm(self, _req, res):
        """ Starts the Vodafone Arm mission """
        new_mission = VodafoneArmMission("vodafone_arm_mission", robot=self.robot_bridge)
        self.load_mission(new_mission)
        res.success = True
        res.message = "Starting Vodafone Arm Mission"
        return res

    def handle_start_testing(self, _req, res):
        """ Starts a test mission, for testing only """
        new_mission = ElevatorMission("elevator_mission", map_manager=self.map_manager, elevator_api=self.elevator)
        self.load_mission(new_mission)
        res.success = True
        res.message = "Starting Elevator Mission"
        return res

    def handle_start_docking(self, _req, res):
        """ Starts a docking mission, for testing only """
        new_mission = DockMission("dock_mission", navigator=self.navigator, robot=self.robot_bridge, map_manager=self.map_manager)
        self.load_mission(new_mission)
        res.success = True
        res.message = "Starting Docking Mission"
        return res

    def handle_start_undocking(self, _req, res):
        """ Starts a undocking mission, for testing only """
        new_mission = UndockMission("undock_mission", robot=self.robot_bridge)
        self.load_mission(new_mission)
        res.success = True
        res.message = "Starting Undocking Mission"
        return res

    def handle_start_pickup(self, _req, res):
        """ Starts a pickup mission. Assumes a single pickup location """
        new_mission = LoadCargoMission("pickup_mission", location="pharmacy", map_name="floor-1", navigator=self.navigator, robot=self.robot_bridge, map_manager=self.map_manager, elevator_api=self.elevator)
        self.load_mission(new_mission)
        # Mission 1 - Pharmacy to Floor1
        delivery_mission_1 = CUFDelivery("delivery_mission_1", location="elevator_in", map_name="floor1", navigator=self.navigator, robot=self.robot_bridge, map_manager=self.map_manager, elevator_api=self.elevator, door_api=self.door_api, trays=["bottom_tray"])
        self.load_mission(delivery_mission_1)
        # Mission 2 - Floor1 to UCIP to Floor5
        delivery_mission_2 = CUFDelivery2("delivery_mission_2", location="ucip", map_name="floor1", navigator=self.navigator, robot=self.robot_bridge, map_manager=self.map_manager, elevator_api=self.elevator, door_api=self.door_api,trays=["front"])
        self.load_mission(delivery_mission_2)
        # Mission 3 - Floor5 to Balcao A to Elevator
        delivery_mission_3 = CUFDelivery3("delivery_mission_3", location="balcao_a", map_name="floor5", navigator=self.navigator, robot=self.robot_bridge, map_manager=self.map_manager, elevator_api=self.elevator, door_api=self.door_api,trays=["front"])
        self.load_mission(delivery_mission_3)

        res.success = True
        res.message = "Starting PickUp Mission"
        return res

    def handle_start_delivery(self, _req, res):
        """ Starts a delivery mission. Assumes pre-defined locations for deliveries """
        # delivery_mission_1 = DeliverCargoMission("delivery_mission_1", location="idmind", map_name="floor2", navigator=self.navigator, robot=self.robot_bridge, map_manager=self.map_manager, elevator_api=self.elevator, trays=["front"])
        delivery_mission_2 = DeliverCargoMission("delivery_mission_2", location="management", map_name="floor0", navigator=self.navigator, robot=self.robot_bridge, map_manager=self.map_manager, elevator_api=self.elevator, trays=["top_tray"])
        # delivery_mission_3 = DeliverCargoMission("delivery_mission_3", location="idmind", map_name="floor0", navigator=self.navigator, robot=self.robot_bridge, map_manager=self.map_manager, elevator_api=self.elevator, trays=["bottom_tray"])
        # self.load_mission(delivery_mission_1)
        self.load_mission(delivery_mission_2)
        # self.load_mission(delivery_mission_3)
        
        res.success = True
        res.message = "Starting Delivery Missions"
        return res

    def update_joy(self, msg):
        self.joy = msg
        # B is pressed for greet-bye
        if self.joy.buttons[1]:
            self.robot_bridge.set_hri_preset("GREET_BYE")
            self.robot_bridge.update_mission_state({"mission_state": "Saying hello", "mission_state_message": "Saying hello."})
    
    ######################
    #    MAIN METHODS    #
    ######################
    def load_mission(self, mission, clear=False):
        """ Load a new mission to queue"""
        self.get_logger().info("Loading new mission")
        if clear:
            self.missions.cancel_all()
        
        self.missions.add_mission(mission)
        return True
    
    def shutdown(self):
        return True
    
    def start(self):
        """ Main method to be called periodically """
        if self.in_loop:
            return True
        else:
            self.in_loop = True

        if rclpy.ok():
            current_docked = self.robot_bridge.is_docked()
            if not self.docked and current_docked:
               self.relocate_to_dock = True
            elif not current_docked:
                self.relocate_to_dock = False
 
            if self.relocate_to_dock:
                curr_map = self.map_manager.get_map()
                if curr_map is not None:
                    loc = self.map_manager.get_location("dock", map_name=curr_map)
                    p = PoseWithCovarianceStamped()
                    p.header            = loc["pose"].header
                    p.pose.pose         = loc["pose"].pose
                    p.pose.covariance   = 0.1*np.identity(6).flatten()
                    self.map_manager.set_pose(p)
                    self.relocate_to_dock = False
                
            if self.cancel_mission_flag:
                self.get_logger().warn("Cancelling all missions")
                self.missions.cancel_all()
                self.cancel_mission_flag = False

            if len(self.missions.get_mission_list()) > 0:
                # CHECK IF ROBOT WAS DOCKED unexpectedly AND CANCEL ALL MISSIONS
                m = self.missions.get_current_mission()
                t = m.get_current_task()
                if (not self.docked and current_docked):                    
                    if type(t) != DockTask:
                        self.cancel_mission_flag = True
                else:    
                    st = m.get_status()
                    if st == MissionStatus.UNKNOWN:
                        self.get_logger().info("Starting mission {}".format(m.name))
                        m.start()
                    elif st in [MissionStatus.STARTED, MissionStatus.RUNNING]:
                        self.get_logger().debug("Running mission {}".format(m.name))
                        m.update()
                    else:
                        self.get_logger().warn("Finished mission {}".format(m.name))
                        self.missions.remove_mission(0)
            else:
                # Choose what happens when there are no missions- idle or docking
                # if self.robot_bridge.get_mission_state() != "IDLE":
                #     self.robot_bridge.set_hri_preset("IDLE")
                #     mission_state = {}
                #     mission_state["mission_state"] = "START"
                #     mission_state["mission_state_message"] = "Ready to start"
                #     mission_state["warning_state"] = "NONE"
                #     mission_state["warning_state_message"] = ""                    
                #     self.robot_bridge.update_mission_state(mission_state)
                pass
                # if current_docked is not None and not current_docked:
                #     new_mission = DockMission("dock_mission", navigator=self.navigator, robot=self.robot_bridge, map_manager=self.map_manager)
                #     self.load_mission(new_mission)

            self.docked = current_docked
            self.robot_bridge.publish_mission_state()

        self.in_loop = False
        
        return True
 
 
def main(args=None):
    rclpy.init(args=args)
        
    try:
        executor = MultiThreadedExecutor(num_threads=6)
        robot_bridge = RobotBridge(node_name="robot_api")
        map_manager = MapManager(node_name="map_manager")
        elevator_api = ElevatorAPI(node_name="elevator_api")
        door_api = DoorAPI(node_name="door_api")
        mission_controller = MissionController(robot_bridge, map_manager, elevator_api, door_api, node_name="mission_controller")
        executor.add_node(mission_controller)
        executor.add_node(robot_bridge)
        executor.add_node(map_manager)
        executor.add_node(elevator_api)
        executor.add_node(door_api)
        try:
            executor.spin()
        except KeyboardInterrupt:
            print('\033[91m'+"Shutting down Mission Controller"+'\033[0m')
        finally:
            executor.shutdown()
            mission_controller.shutdown()
            mission_controller.destroy_node()
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
