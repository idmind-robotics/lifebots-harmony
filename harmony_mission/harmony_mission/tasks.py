#! /usr/bin/env python3

import time
import numpy as np
from enum import Enum
from colorama import Fore, Back, Style

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from squaternion import Quaternion

from .robot_navigator import BasicNavigator, NavigationResult
from .robot_bridge import RobotBridge
from .map_manager import MapManager
from .elevator_api import ElevatorAPI

ALARM_TIMEOUT=5.0

##############################
###      TASKS CLASSES     ###
##############################
class TaskStatus(Enum):
    FAILED = -1
    UNKNOWN = 0
    STARTED = 1
    RUNNING = 2
    CANCELED = 3
    FINISHED = 4

class Task(object):
    def __init__(self, name, task_type):
        """
        Initializes a new instance of the Task class.

        Parameters:
        name (str): The name of the task.
        task_type (str): The type of the task.

        Returns:
        None
        """
        super().__init__()  # Calls the parent class's constructor
        self.name = name  # Stores the name of the task
        self.task_type = task_type  # Stores the type of the task
        self.status = TaskStatus.UNKNOWN  # Initializes the status of the task to UNKNOWN

    def start(self):
        """ Starts the task:
            - updates the task state
        """
        self.print("{} Task {}: starting".format(self.task_type, self.name))
        self.status = TaskStatus.STARTED
        return True

    def print(self, msg, color="bold"):
        end = '\033[0m'
        cd = '\033[1m' if color=="bold" else ('\033[92m' if color=="green" else '\033[91m')
        print(cd+msg+end)
    
    def update(self):
        """ Updated the status of the task - REIMPLEMENT """
        self.status = TaskStatus.RUNNING
        return True

    def stop(self, success=True):
        """ Stops the Task """
        self.print("{} Task {}: stopping".format(self.task_type, self.name))
        self.status = TaskStatus.FINISHED if success else TaskStatus.FAILED
        return True
    
    def cancel(self):
        """ Cancels the task. Might require a call for update method """
        self.print("{} Task {}: cancelled".format(self.task_type, self.name), color="red")
        self.status = TaskStatus.CANCELED
        return True

    def get_status(self):
        return self.status

    def is_running(self):
        return self.status==TaskStatus.RUNNING

    def is_finished(self):
        return self.status in [TaskStatus.FINISHED, TaskStatus.FAILED, TaskStatus.CANCELED]
    
    def __str__(self):
        msg = "{} Task {}: {}".format(self.task_type, self.name, self.status)
        return msg

##################################
###      TASKS DEFINITIONS     ###
##################################
class WaitTask(Task):
    """ Implements a Waiting Task """
    def __init__(self, name, wait_time):
        super().__init__(name=name, task_type="WaitTask")
        self.start_time = None
        self.wait_time = wait_time
    
    def start(self):
        super().start()
        self.start_time = time.time()
        
    def update(self):
        if time.time() - self.start_time >= self.wait_time:
            self.stop(success=True)            
        else:
            self.status = TaskStatus.RUNNING
        return True
        
class WaitInterfaceTask(Task):
    """ Implements a Waiting for Interface Task """
    def __init__(self, name, robot_api: RobotBridge):
        super().__init__(name=name, task_type="WaitInterfaceTask")
        self.robot_api = robot_api
    
    def start(self):
        super().start()
        self.robot_api.clear_wait_task = False
        
    def update(self):
        if self.robot_api.get_clear_wait_task():
            self.stop(success=True)            
        else:
            self.status = TaskStatus.RUNNING
        return True

class DockTask(Task):
    def __init__(self, name: str, robot_bridge: RobotBridge, map_manager: MapManager):
        super().__init__(name=name, task_type="DockTask")
        self.robot_bridge = robot_bridge
        self.map_manager = map_manager
    
    def start(self):
        super().start()
        self.robot_bridge.set_hri_preset("NAVIGATING")
        self.robot_bridge.update_mission_state({"mission_state": "NAVIGATING", "mission_state_message": "Docking"})
        return True

    def update(self):
        # Check state of robot charging/docking
        docked = self.robot_bridge.is_docked()
        charging = self.robot_bridge.is_charging()
        bumpers = self.robot_bridge.bumpers_active()
        
        # Check for bumpers. If not charging, leave and fail. Mission handler should retry the docking.
        if bumpers:
            if not docked:
                self.robot_bridge.motor_takeover(True)
                time.sleep(1.0)
                self.robot_bridge.move_forward(dist=0.5)
                self.stop(success=False)
                return True
            else:
                self.robot_bridge.motor_takeover(False)
                self.stop(success=True)
                return True

        else:
            if docked:
                # If the robot is docked, the mission was successfull
                self.robot_bridge.move_forward(dist=0.0)
                self.stop()
                return True
            else:
                # If the robot is not docked, move backwards (there is no checking for alignment...)
                self.robot_bridge.move_forward(dist=-0.1)
                return True
    
    def stop(self, success=True):
        super().stop(success=success)
        return True

class UndockTask(Task):
    def __init__(self, name: str, robot_bridge: RobotBridge):
        super().__init__(name=name, task_type="UndockTask")
        self.last_motor_takeover = None
        self.robot_bridge = robot_bridge
    
    def start(self):
        super().start()
        self.robot_bridge.update_mission_state({"mission_state": "NAVIGATING", "mission_state_message": "Undocking"})
        self.robot_bridge.set_hri_preset("NAVIGATING")
        return True
    
    def update(self):
        # Check state of robot charging/docking
        docked = self.robot_bridge.is_docked()
        charging = self.robot_bridge.is_charging()
        
        if not docked:
            self.robot_bridge.motor_takeover(False)
            self.stop()
            return True
        else:
            if charging:
                self.robot_bridge.undock()
                time.sleep(1.0)
                return True
            else:
                # Check bumpers, sometimes they get activated
                if self.robot_bridge.bumpers_active() and (self.last_motor_takeover is None or (time.time()-self.last_motor_takeover > 10.0)):
                    self.robot_bridge.motor_takeover(True)
                    self.last_motor_takeover = time.time()
                    return True
                else:
                    # If the robot is not charging, move forward
                    self.robot_bridge.move_forward(dist=0.5)
                    return True

class GoToTask(Task):
    """ Implements a GoTo Task (communicates with ROS2 Navigation)"""
    def __init__(self, name: str, destination: str, map_name: str, robot_bridge:RobotBridge, navigator: BasicNavigator, map_manager: MapManager):
        super().__init__(name=name, task_type="GoToTask")
        self.map_name = map_name
        self.destination = destination
        self.robot_bridge = robot_bridge
        self.navigator = navigator
        self.map_manager = map_manager
        self.active_navigation = False

    def destination2pose(self, destination, floor=None) -> PoseStamped:
        """ A destination string is converted to a Map Pose """
        loc = self.map_manager.get_location(destination, floor)
        return loc["pose"]

    def start(self):
        super().start()
        self.active_navigation = False
        self.robot_bridge.update_mission_state({
            "mission_state": "NAVIGATING", "mission_state_message": "Navigating to {}".format(self.destination),
            "warning_state": "NONE", "warning_state_message": ""})
        self.robot_bridge.set_hri_preset("NAVIGATING")
        return True
    
    def cancel(self):
        # CANCEL NAVIGATION
        self.navigator.cancelNav()
        super().cancel()
        return True

    def set_navigation_tasks(self):
        """ This method should create new tasks if the final goal is not in the current map """
        self.navigator.clearAllCostmaps()
        self.navigator.goToPose(self.destination2pose(self.destination, self.map_name))
        
    def update(self):
        """ Checks if navigation is completed (successful or not) """
        if self.get_status() == TaskStatus.CANCELED:
            self.navigator.cancelNav()
        
        if not self.active_navigation:
            self.set_navigation_tasks()
            self.active_navigation = True

        if self.navigator.isNavComplete():
            self.active_navigation = False
            fb = self.navigator.getResult()
            if fb == NavigationResult.SUCCEEDED:
                self.print("{} Task {}: finished".format(self.task_type, self.name), color="green")
                self.stop(success=True)
            else:
                if not self.close_enough():
                    self.print("{} Task {}: failed. Retrying.".format(self.task_type, self.name))
                    self.robot_bridge.update_mission_state({
                        "mission_state": "ALARM", "mission_state_message": "Navigating to {}".format(self.destination), 
                        "warning_state": "BLOCKED", "warning_state_message": "Navigation failed."})
                    self.start()
                else:
                    self.print("{} Task {}: close enough and finished".format(self.task_type, self.name), color="green")
                    self.stop(success=True)
        else:
            # self.print("{} Task {}: updating".format(self.task_type, self.name))
            self.robot_bridge.update_mission_state({"mission_state": "NAVIGATING", "mission_state_message": "Navigating to {}".format(self.destination)})
            pass
            
        return True

    def close_enough(self):
        try:
            goal_pose = self.destination2pose(self.destination, self.map_name)
            robot_pose = self.map_manager.get_pose(robot_frame="/harmony/base_link")
            
            goal_x = goal_pose.pose.position.x
            goal_y = goal_pose.pose.position.y
            robot_x = robot_pose.pose.pose.position.x
            robot_y = robot_pose.pose.pose.position.y
            dist = np.linalg.norm([goal_x-robot_x, goal_y-robot_y])
            if dist < 0.2:
                return True 
            return False
        except Exception:
            return False


class LoadTask(Task):
    """ 
        Implements a Load Task.
        Upon arriving at a location (TODO: check location), wait for user request to unlock drawers.
        Wait for user confirmation that load is complete.
        Check all drawers are locked. If not, sound the "alarm" until they are.
        Complete the task.
    """
    def __init__(self, name: str, robot_bridge: RobotBridge):
        super().__init__(name=name, task_type="LoadTask")
        self.robot_bridge = robot_bridge
        self.load_complete = False
        self.last_alarm = None

    def start(self):
        super().start()
        self.robot_bridge.clear_lock_goals()
        self.load_complete = False
        # Signal web interface to show unlock drawers menu?
        self.robot_bridge.set_hri_preset("GREET_HI_LOADING")
        time.sleep(2.0)
        self.robot_bridge.set_hri_preset("READY_TO_LOAD_UNLOAD")
        self.robot_bridge.update_mission_state({"mission_state": "READY_TO_LOAD_UNLOAD", "mission_state_message": "Loading at pickup location"})
        return True

    def update(self):
        """
            Waits for user request to opens assigned drawers
            Waits for user confirmation of load complete
            Checks drawers are closed
                - Warns if something is wrong
                - Completes the task
        """
        super().update()

        # Wait for user to complete Load. 
        if not self.load_complete and self.robot_bridge.get_mission_state() != "LOADING_COMPLETED":
            state = self.robot_bridge.get_lock_status()
            goal = self.robot_bridge.get_lock_goals()
            for lock in ["front", "top_tray", "bottom_tray"]:
                if (goal[lock]["locked"] is not None) and (state[lock]["locked"] != goal[lock]["locked"]):
                    if state[lock]["ready"]:
                        self.print("{} Task {}: opening {} Lock".format(self.task_type, self.name, lock))
                        self.robot_bridge.open_lock(lock)
                        self.robot_bridge.clear_lock_goals(locks=[lock])
                        # No rush, don't block, wait for next iteration
                        break
            return TaskStatus.RUNNING        
        
        # Save for future iterations
        self.load_complete = True

        # Check if all drawers are closed
        all_closed = True
        state = self.robot_bridge.get_lock_status()
        for lock in ["front", "top_tray", "bottom_tray"]:
            all_closed = all_closed and state[lock]["locked"]
        if not all_closed:
            # sound warning
            curr_time = time.time()
            mission_state = {}
            mission_state["mission_state"] = "ALARM"
            msg = "Alarm: "
            for name, locked in state.items():
                if not locked:
                    msg+="{} storage unlocked | ".format(name)
            msg = msg[:-3]
            mission_state["mission_state_message"] = msg
            mission_state["warning_state"] = "TRAY_ALARM"
            mission_state["warning_state_message"] = msg
            self.robot_bridge.update_mission_state(mission_state)
            if self.last_alarm is None or curr_time-self.last_alarm > ALARM_TIMEOUT:
                self.print("{} Task {}: Locks are not closed".format(self.task_type, self.name))
                self.robot_bridge.set_hri_preset("ALARM")
                self.last_alarm = curr_time
            
            return TaskStatus.RUNNING
        else:
            self.robot_bridge.set_hri_preset("LOADING_COMPLETED")
            mission_state = {}    
            mission_state["mission_state"] = "LOADING_COMPLETED"
            mission_state["mission_state_message"] = "Loading complete."
            mission_state["warning_state"] = "NONE"
            mission_state["warning_state_message"] = ""
            self.print("{} Task {}: finished".format(self.task_type, self.name), color="green")
            self.stop()


class DeliverTask(Task):
    """ 
        Implements a Deliver Task.
        Upon arriving at a location (TODO: check location):
            - If the delivery is autonomous, perform the arm delivery routine
            - If the delivery is assisted, wait for user request to unlock drawer.
        When delivery is complete, check all locks are locked or sound a warning
        Complete the task.
    """
    def __init__(self, name: str, robot_bridge: RobotBridge, trays: str, autonomous=False):
        super().__init__(name=name, task_type="DeliverTask")
        if autonomous and "top_tray" not in trays :
            raise Exception("Autonmous delivery is only accepted for Top Tray")
        self.robot_bridge = robot_bridge
        self.last_alarm = None
        self.deliver_complete = False
        self.keep_unlocked = False
        self.trays = trays
        self.autonomous = autonomous
        self.autonomous_open_drawer = False
        self.autonomous_deliver = False
        self.autonomous_close_drawer = False
        self.curr_animation = None

    def start(self):
        super().start()
        self.deliver_complete = False
        self.keep_unlocked = True
        self.autonomous_open_drawer = False
        self.autonomous_deliver = False
        self.autonomous_close_drawer = False
        
        # Signal web interface to show unlock drawers menu?
        self.robot_bridge.set_hri_preset("READY_TO_LOAD_UNLOAD")
        self.robot_bridge.update_mission_state({"mission_state": "ARRIVED_DELIVERY", "mission_state_message": "Ready to make delivery."})
        return True

    def update(self):
        """
            If autonomous, make the arm routine (without blocking)
            If not, wait for user request to unlock drawer
            Checks drawers are closed
                - Warns if something is wrong
                - Completes the task
        """
        super().update()

        if self.autonomous:
            return self.autonomous_delivery()
        else:
            return self.assisted_delivery()
    
    def autonomous_delivery(self):
        # Get data
        lock_state = self.robot_bridge.get_lock_status()
        # arm_ready = self.robot_bridge.arm_ready()
        arm_state = self.robot_bridge.get_arm_state()
        
        if not self.robot_bridge.get_arm_power_status():
            self.print("Arm is not powered", color="red")
            return TaskStatus.RUNNING

        if self.curr_animation is None:
            if lock_state["top_tray"]["ready"]:
                self.print("{} Task {}: opening top_tray Lock (try1)".format(self.task_type, self.name))
                self.robot_bridge.open_lock_not_block("top_tray")
                self.robot_bridge.clear_lock_goals(locks=["top_tray"])
            # Start by opening the drawer
            if self.robot_bridge.arm_animation("open_drawer"):
                self.print("Starting open_drawer animation")
                self.curr_animation = "open_drawer"
                self.print("{} Task {}: opening top_tray Lock (try2)".format(self.task_type, self.name))
                self.robot_bridge.open_lock_not_block("top_tray")
                self.robot_bridge.clear_lock_goals(locks=["top_tray"])
            return TaskStatus.RUNNING
        elif self.curr_animation == "open_drawer":
            # if lock_state["top_tray"]["ready"]:
            self.print("{} Task {}: opening top_tray Lock (try3)".format(self.task_type, self.name))
            self.robot_bridge.open_lock_not_block("top_tray")
            self.robot_bridge.clear_lock_goals(locks=["top_tray"])
            if arm_state == "None":
                self.print("Finished open_drawer animation")
                if self.robot_bridge.arm_animation("deliver"):
                    self.print("Starting deliver animation")
                    self.curr_animation = "deliver"
            return TaskStatus.RUNNING
        elif self.curr_animation == "deliver":
            if arm_state == "None":
                self.print("Finished deliver animation")
                if self.robot_bridge.arm_animation("close_drawer"):
                    self.print("Starting close_drawer animation")
                    self.curr_animation = "close_drawer"
            return TaskStatus.RUNNING
        elif self.curr_animation == "close_drawer":
            if arm_state == "None":
                self.print("Finished close_drawer animation")
                self.curr_animation = None
                self.stop(success=True)
            else:
                return TaskStatus.RUNNING
            
    def assisted_delivery(self):
        # Wait for user to confirm presence. 
        if not self.deliver_complete and self.robot_bridge.get_mission_state() != "DELIVERY_COMPLETED":
            if self.robot_bridge.get_mission_state() == "DELIVERY_READY":
                state = self.robot_bridge.get_lock_status()
                goal = self.robot_bridge.get_lock_goals()
                for lock in self.trays:
                    if (goal[lock]["locked"] is not None) and (state[lock]["locked"] != goal[lock]["locked"]):
                        if state[lock]["ready"]:
                            self.print("{} Task {}: opening {} Lock".format(self.task_type, self.name, lock))
                            self.robot_bridge.open_lock(lock)
                            self.robot_bridge.clear_lock_goals(locks=[lock])
                            # No rush, don't block, wait for next iteration
                            break
            return TaskStatus.RUNNING
        
        # Save for future iterations
        self.deliver_complete = True

        # Check if all drawers are closed
        all_closed = True
        state = self.robot_bridge.get_lock_status()
        for lock in ["front", "top_tray", "bottom_tray"]:
            all_closed = all_closed and state[lock]["locked"]
        if not all_closed:
            # sound warning
            curr_time = time.time()
            mission_state = {}
            mission_state["mission_state"] = "ALARM"
            msg = "Alarm: "
            for name, locked in state.items():
                if not locked:
                    msg+="{} storage unlocked | ".format(name)
            msg = msg[:-3]
            mission_state["mission_state_message"] = msg
            mission_state["warning_state"] = "TRAY_ALARM"
            mission_state["warning_state_message"] = msg
            self.robot_bridge.update_mission_state(mission_state)
            if self.last_alarm is None or curr_time-self.last_alarm > ALARM_TIMEOUT:
                self.print("{} Task {}: Locks are not closed".format(self.task_type, self.name))
                self.robot_bridge.set_hri_preset("ALARM")
                self.last_alarm = curr_time
            return TaskStatus.RUNNING
        else:
            self.robot_bridge.set_hri_preset("LOADING_COMPLETED")
            mission_state = {}    
            mission_state["mission_state"] = "LOADING_COMPLETED"
            mission_state["mission_state_message"] = "Delivery complete."
            mission_state["warning_state"] = "NONE"
            mission_state["warning_state_message"] = ""
            self.print("{} Task {}: finished".format(self.task_type, self.name), color="green")   
            time.sleep(1.0)         
            self.stop()

class TakeElevatorTask(Task):
    def __init__(self, name:str, start_floor, destiny_floor, robot_bridge: RobotBridge, navigator: BasicNavigator, map_manager:MapManager, elevator_api: ElevatorAPI):
        super().__init__(name=name, task_type="TakeElevatorTask")
        self.start_floor = start_floor
        self.destiny_floor = destiny_floor
        self.elevator = elevator_api
        self.map_manager = map_manager
        self.robot = robot_bridge
        self.navigator = navigator
        self.navigating = False
        self.changed_map = False
        self.task_state = None

    def start(self):
        super().start()
        self.task_state = "call_elevator"
        return True

    def update(self):
        """ 
            This method should not be blocking.
            Call elevator. Wait for doors to open. Enter elevator.
            Wait for doors to close. Wait for doors to open. Leave elevator if at right floor.
        """
        super().update()

        if self.task_state == "call_elevator":
            if self.elevator.call_elevator(self.start_floor):
                self.task_state = "wait_elevator"
        else:
            ef = self.elevator.is_elevator_finished()
            if ef:
                self.map_manager.load_map(self.destiny_floor)
                loc = self.map_manager.get_location("elevator_out", self.destiny_floor)
                self.map_manager.set_pose(loc["pose"])
                self.print("Map changed and robot relocated")
                self.changed_map = True
                self.stop(success=True)
        
        return True            
        # if self.task_state == "call_elevator":
        #     if self.elevator.call_elevator(self.start_floor):
        #         self.task_state = "wait_elevator"
        # elif self.task_state == "wait_elevator":
        #     if not self.elevator.get_door_open():
        #         return True
        #     else:
        #         self.task_state = "enter_elevator"
        # elif self.task_state == "enter_elevator":
        #     if self.enter_elevator():
        #         self.task_state = "use_elevator"
        #         self.elevator.close_door()
        # elif self.task_state == "use_elevator":
        #     if not self.changed_map:
        #         self.map_manager.load_map(self.destiny_floor)
        #         loc = self.map_manager.get_location("elevator_16", self.destiny_floor)
        #         self.map_manager.set_pose(loc["pose"])
        #         self.print("Map changed and robot relocated")
        #         self.changed_map = True
        #         self.print("Waiting for elevator door to open", color="green")
        #     if not self.elevator.get_door_open():
        #         return True
        #     else:
        #         self.task_state = "exit_elevator"
        # elif self.task_state == "exit_elevator":
        #     if self.exit_elevator():
        #         self.stop(success=True)
        # return True
    
    # def enter_elevator(self):
        
    #     if not self.navigating:
    #         self.print("Entering elevador", color="green")
    #         map_name = self.map_manager.get_map()
    #         loc = self.map_manager.get_location("elevator_16", map_name)
    #         self.navigator.clearAllCostmaps()
    #         time.sleep(0.5)
    #         self.navigator.goToPose(loc["pose"])
    #         self.navigating = True

    #     if self.navigator.isNavComplete():
    #         self.navigating = False
    #         fb = self.navigator.getResult()
    #         if fb == NavigationResult.SUCCEEDED:
    #             return True
    #         else:
    #             return False

    # def exit_elevator(self):

    #     if not self.navigating:
    #         self.print("Exiting elevador", color="green")
    #         map_name = self.map_manager.get_map()
    #         loc = self.map_manager.get_location("elevator_out", map_name)
    #         self.navigator.clearAllCostmaps()
    #         time.sleep(0.5)
    #         self.navigator.goToPose(loc["pose"])
    #         self.navigating = True

    #     if self.navigator.isNavComplete():
    #         self.navigating = False
    #         fb = self.navigator.getResult()
    #         if fb == NavigationResult.SUCCEEDED:
    #             return True
    #         else:
    #             return False

    # def get_middle_scan(self, scan):
    #     """ Auxiliary mission to return the middle scan """
    #     ang_min = scan.angle_min
    #     ang_inc = scan.angle_increment
    #     idx = None
    #     for i in range(0, len(scan.ranges)):
    #         ang = ang_min + i*ang_inc
    #         if ang > 0:
    #             return scan.ranges(idx)
                
    # def get_doorway(self, scan):
    #     # obstacles = []
    #     # ang = scan.angle_min
    #     # for idx in range(0, len(scan.ranges)):
    #     #     ang = scan.angle_min+idx*scan.angle_increment
    #     #     if ang > -1.57 or ang < 1.57:
    #     #         ox = scan.ranges[idx]*np.cos(ang)
    #     #         oy = scan.ranges[idx]*np.sin(ang)
    #     #         obstacles.append([ox, oy, scan.ranges[idx]])
        
    #     # Find front_distance and idx
    #     for idx in range(0, len(scan.ranges)):
    #         ang = scan.angle_min+idx*scan.angle_increment
    #         if ang > 0.0:
    #             front_idx = idx
    #             front_distance = scan.ranges[idx]
    #             break
    #     print("Front IDX: {} of {}".format(front_idx, len(scan.ranges)))
        
    #     # Search for right doorway
    #     right_doorway = None
    #     idx = front_idx
    #     while right_doorway is None:
    #         idx = idx -1
    #         if idx < 0:
    #             break 
    #         ang = scan.angle_min+idx*scan.angle_increment
    #         # print("Checking angle {}".format(ang))
    #         if abs(scan.ranges[idx]-scan.ranges[idx+1]) > 0.2:
    #             right_doorway = [scan.ranges[idx]*np.cos(ang), scan.ranges[idx]*np.sin(ang)]
    #             print("Found doorway at {}".format(right_doorway))
    #     self.print("right: {} | Ang: {}".format(right_doorway, ang*180/np.pi))
        
    #     # Search for left doorway
    #     left_doorway = None
    #     idx = front_idx
    #     while left_doorway is None:
    #         idx = idx +1
    #         if idx > len(scan.ranges):
    #             break 
    #         ang = scan.angle_min+idx*scan.angle_increment
    #         # print("Checking angle {}".format(ang))
    #         if abs(scan.ranges[idx]-scan.ranges[idx-1]) > 0.2:
    #             left_doorway = [scan.ranges[idx]*np.cos(ang), scan.ranges[idx]*np.sin(ang)]
    #             print("Found doorway at {}".format(left_doorway))
    #     self.print("left: {} | Ang: {}".format(left_doorway, ang*180/np.pi))

    #     return [left_doorway, right_doorway]
        
class OpenDoorTask(Task):
    def __init__(self, name, door_api):
        super().__init__(name, task_type="OpenDoorTask")
        self.door_api = door_api
        self.door_open = False
        self.start_waiting = None

    def start(self):
        super().start()
        self.door_open = False
        self.start_waiting = time.time()

    def update(self):
        """ 
            This method should not be blocking.
            Wait for doors to open.
        """
        super().update()
        # now = time.time()
        # if now - self.start_waiting > 5.0:
        #     self.door_open = True
        # else:
        #     self.door_open = False
            
        self.door_open = self.door_api.get_door_state()
        if self.door_open:
            self.print("Door is now open")
            self.stop(success=True)
        
        return True
            

class TriggerBehaviourTask(Task):
    """
        This task will trigger the HRI Obstructed Sequence and wait for its finish
    """
    def __init__(self, name, robot_api: RobotBridge, behaviour: str):
        super().__init__(name, task_type="TriggerObstruction")
        self.robot = robot_api
        self.start_time = None
        self.behaviour = behaviour

    def start(self):
        super().start()
        self.start_time = time.time()
        self.robot.set_hri_preset(self.behaviour)
    
    def update(self):
        super().start()
        if time.time() - self.start_time > 4.0:
            self.stop(success=True)
        
    
class TriggerObstructionTask(Task):
    """
        This task will trigger the HRI Obstructed Sequence and wait for its finish
    """
    def __init__(self, name, robot_api: RobotBridge):
        super().__init__(name, task_type="TriggerObstruction")
        self.hri_state = None
        self.robot = robot_api
        self.start_time = None
        self.triggered = False
        self.moved_forward = False

    def start(self):
        super().start()
        self.start_time = time.time()

    def update(self):
        super().update()

        if not self.triggered:
            self.robot.set_hri_preset("UNLOADING_WITH_ARM")
            mission_state = {}
            mission_state["mission_state"] = "Navigating"
            mission_state["mission_state_message"] = "Navigating to Elevator at Floor -1"
            mission_state["warning_state"] = "OBSTRUCTED"
            mission_state["warning_state_message"] = "Path obstruciton cleared, will continue soon."
            self.robot.update_mission_state(mission_state)
            self.triggered = True
        else:
            if time.time() - self.start_time > 10.0:
                self.stop(success=True)
            else:
                if not self.moved_forward:
                    self.robot.move_forward(-0.08, 1.0)
                    time.sleep(0.5)
                    self.robot.move_forward(0.08, 1.0)
                    time.sleep(0.5)
                    self.robot.move_forward(-0.08, 1.0)
                    time.sleep(0.5)
                    self.robot.move_forward(0.08, 1.0)
                    self.moved_forward = True

class TriggerMitigationTask(Task):
    """
        This task will trigger the HRI Obstructed Sequence and wait for its finish
    """
    def __init__(self, name, robot_api: RobotBridge):
        super().__init__(name, task_type="MitigationTask")
        self.hri_state = None
        self.robot = robot_api
        self.start_time = None
        self.triggered = False
        self.moved_forward = False

    def start(self):
        super().start()
        self.start_time = time.time()

    def update(self):
        super().update()

        if not self.triggered:
            self.robot.set_hri_preset("FUNNY_FART_WHISTLE")
            mission_state = {}
            mission_state["mission_state"] = "Navigating"
            mission_state["mission_state_message"] = "Navigating to UCIP at Floor 1"
            mission_state["warning_state"] = "OBSTRUCTED"
            mission_state["warning_state_message"] = "Path obstruciton cleared, will continue soon."
            self.robot.update_mission_state(mission_state)
            self.triggered = True
        else:
            if time.time() - self.start_time > 5.0:
                self.stop(success=True)
            else:
                if not self.moved_forward:
                    self.robot.move_forward(-0.15, 1.0)
                    time.sleep(0.5)
                    self.moved_forward = True
