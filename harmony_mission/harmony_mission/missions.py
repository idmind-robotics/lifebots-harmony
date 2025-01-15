#! /usr/bin/env python3

import rclpy

import time
from .tasks import *
from enum import Enum

################################
###      MISSION CLASSES     ###
################################
class MissionStatus(Enum):
    FAILED = -1
    UNKNOWN = 0
    STARTED = 1
    RUNNING = 2
    CANCELLED = 3
    FINISHED = 4


class Mission(object):
    """ Generic Mission class """
    def __init__(self, name, mission_type):
        """ Initialize the Mission class """
        super().__init__()
        self.name = name
        self.mission_type = mission_type
        self.status = MissionStatus.UNKNOWN
        self.tasks = []
        self.current_task_idx = -1

    def start(self):
        """ Starts the mission:
            - starts the first task 
            - updates the mission state
            - stops if there are no task listed
        """
        if len(self.tasks) > 0:
            print("{} Mission {}: starting".format(self.mission_type, self.name))
            for t in self.tasks:
                t.status = TaskStatus.UNKNOWN
            self.current_task_idx = 0
            self.status = MissionStatus.STARTED
            return True
        else:
            print("{} Mission {}: there are no tasks".format(self.mission_type, self.name))
            self.stop()
            return False

    def update(self):
        """ Updated the status of the mission, by going through tasks """
        # Update current task and check status of current task
        t = self.tasks[self.current_task_idx]
        if t.status == TaskStatus.UNKNOWN:
            t.start()
        elif t.status in [TaskStatus.STARTED, TaskStatus.RUNNING]:
            self.status = MissionStatus.RUNNING
            t.update()
        elif t.status == TaskStatus.FINISHED:
            # If this was the last task, lets finish it
            if self.current_task_idx >= len(self.tasks)-1:
                self.stop(success=True)
            else:
                self.current_task_idx += 1
        elif t.status == TaskStatus.FAILED:
            print("{} Mission {}: failed".format(self.mission_type, self.name))
            self.status = MissionStatus.FAILED
        else:
            print("{} Mission {}: unknown task status".format(self.mission_type, self.name))
            return False
        return True

    def stop(self, success=True):
        """ Stops the mission """
        print("{} Mission {}: stopping".format(self.mission_type, self.name))
        self.current_task_idx = -1
        self.status = MissionStatus.FINISHED if success else MissionStatus.FAILED
        return True
    
    def cancel(self):
        """ Cancel the current mission by cancelling all remaining tasks """
        for t in self.tasks:
            if t.get_status() in [TaskStatus.UNKNOWN, TaskStatus.STARTED,TaskStatus.RUNNING]:
                t.cancel()
        self.status = MissionStatus.CANCELLED
        return True

    def get_status(self):
        return self.status

    def is_running(self):
        return self.status==MissionStatus.RUNNING

    def is_finished(self):
        return self.status in [MissionStatus.FINISHED, MissionStatus.FAILED]

    def get_current_task(self):
        return self.tasks[self.current_task_idx]

    def __str__(self):
        msg = "{} Mission {}: ".format(self.mission_type, self.name)
        if self.current_task_idx >= 0:
            t = self.tasks[self.current_task_idx]
            msg += "{}".format(t)
        else:
            msg += "{}".format(self.status)
        return msg


class MissionQueue(object):
    """ A class to handle a list of missions and required methods """
    def __init__(self):
        super().__init__()
        self.queue = []
    
    def get_current_mission(self) -> Mission | None:
        if len(self.queue) == 0:
            return None
        else:
            return self.queue[0]
    
    def get_mission(self, idx) -> Mission | None:
        if idx > len(self.queue)-1:
            return None
        else:
            return self.queue[idx]

    def get_mission_list(self) -> list[Mission]:
        """ Return the list of missions """
        return self.queue

    def add_mission(self, mission):
        """ Adds a new mission to the queue """
        self.queue.append(mission)
        return True

    def remove_mission(self, idx=0):
        """ Removes mission from the queue """
        m = self.get_mission(idx)
        if m is not None:
            if m.get_status() == MissionStatus.RUNNING:
                m.cancel()
            self.queue.pop(idx)
        return True

    def cancel_mission(self, idx):
        """ Cancel mission at position idx """
        m = self.get_mission(idx)
        if m is not None:
            m.cancel()
        self.queue.pop(idx)
        return True
    
    def cancel_all(self):
        """ Cancel all missions in queue. Call to task.update() should be done by Task """
        for m in self.queue:
            m.cancel()
        self.queue = []
        return True

####################################
###      MISSION DEFINITIONS     ###
####################################
class VodafoneArmMission(Mission):
    def __init__(self, name, robot: RobotBridge):
        super().__init__(name, "VodafoneArm")
        self.robot_bridge = robot

        deliver_gift_task = DeliverTask(name="gift_deliver", robot_bridge=self.robot_bridge, trays=["top_tray"], autonomous=True)
        self.tasks.append(deliver_gift_task)
    
    
class VodafoneMission(Mission):
    def __init__(self, name, navigator: BasicNavigator, robot: RobotBridge, map_manager: MapManager):
        super().__init__(name, "GoTo")
        self.robot_bridge = robot
        self.map_manager = map_manager

        self.robot_bridge.enable_alarm(False)
        # Patrol1
        gotopatrol1_task = GoToTask(name="nav2patrol1", map_name="bc", destination="patrol1", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager)
        self.tasks.append(gotopatrol1_task)
        wait_task = WaitTask(name="waiting1", wait_time=1.0)
        self.tasks.append(wait_task)        
        ok1_task = TriggerBehaviourTask(name="play_ok1", behaviour="LOADING_COMPLETED", robot_api=self.robot_bridge)
        self.tasks.append(ok1_task)

        # Patrol2
        gotopatrol2_task = GoToTask(name="nav2patrol2", map_name="bc", destination="patrol2", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager)
        self.tasks.append(gotopatrol2_task)
        wait_task = WaitTask(name="waiting", wait_time=1.0)
        self.tasks.append(wait_task)
        ok2_task = TriggerBehaviourTask(name="play_ok2", behaviour="LOADING_COMPLETED", robot_api=self.robot_bridge)
        self.tasks.append(ok2_task)

        self.robot_bridge.enable_alarm(True)
        
        goto_task = GoToTask(name="nav2sos", map_name="bc", destination="sos_call", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager)
        self.tasks.append(goto_task)
        wait_task = WaitTask(name="waiting", wait_time=1.0)
        self.tasks.append(wait_task)        
        sad_task = TriggerBehaviourTask(name="play_sad", behaviour="ALARM", robot_api=self.robot_bridge)
        self.tasks.append(sad_task)
                        
        # goto2_task = GoToTask(name="nav2present", map_name="bc", destination="presentation", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager)
        # self.tasks.append(goto2_task)
        # happy_task = TriggerBehaviourTask(name="play_happy", behaviour="GREET_BYE", robot_api=self.robot_bridge)
        # self.tasks.append(happy_task)
        # deliver_gift_task = DeliverTask(name="gift_deliver", robot_bridge=self.robot_bridge, trays=["top_tray"], autonomous=True)
        # self.tasks.append(deliver_gift_task)

        wait_for_interface = WaitInterfaceTask(name="waiting_interface", robot_api=self.robot_bridge)
        # wait_task_final = WaitTask(name="waiting_final", wait_time=1.0)
        self.tasks.append(wait_for_interface)
        
        goto3_task = GoToTask(name="nav2final", map_name="bc", destination="final_point", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager)
        self.tasks.append(goto3_task)        
    
    def start(self):
        """ Extends the start() method, to update the mission state in the robot """
        super().start()
        self.robot_bridge.update_mission_state({"mission_state": "START", "mission_state_message": "{} Mission {}: starting".format(self.mission_type, self.name)})
        
class GoToMission(Mission):
    def __init__(self, name, destination, navigator: BasicNavigator, robot: RobotBridge, map_manager: MapManager):
        super().__init__(name, "GoTo")
        self.destination = destination
        self.robot_bridge = robot
        self.map_manager = map_manager
        wait_task = WaitTask(name="waiting", wait_time=2.0)
        self.tasks.append(wait_task)
        goto_task = GoToTask(name="nav2dock", destination=destination, robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager)
        self.tasks.append(goto_task)
    
    def start(self):
        """ Extends the start() method, to update the mission state in the robot """
        super().start()
        self.robot_bridge.update_mission_state({"mission_state": "START", "mission_state_message": "{} Mission {}: starting".format(self.mission_type, self.name)})

class ElevatorMission(Mission):
    def __init__(self, name, map_manager:MapManager, elevator_api:ElevatorAPI):
        super().__init__(name, "Elevator")
        elev_task = TakeElevatorTask(name="take_elevator_task", start_floor=2, destiny_floor=0, map_manager=map_manager, elevator_api=elevator_api)
        self.tasks.append(elev_task)
    
class DockMission(Mission):
    def __init__(self, name, destination, map_name, navigator: BasicNavigator, robot: RobotBridge, map_manager: MapManager):
        super().__init__(name, "Dock")
        self.robot_bridge = robot
        goto_task = GoToTask(name="nav2dock", destination=destination, map_name=map_name, robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager)
        self.tasks.append(goto_task)
        dock_task = DockTask(name="dock", robot_bridge=self.robot_bridge,  map_manager=map_manager)
        self.tasks.append(dock_task)
    
    def start(self):
        """ Extends the start() method, to update the mission state in the robot """
        super().start()
        self.robot_bridge.update_mission_state({"mission_state": "START", "mission_state_message": "{} Mission {}: starting".format(self.mission_type, self.name)})

class UndockMission(Mission):
    def __init__(self, name, robot: RobotBridge):
        super().__init__(name, "Unock")
        self.robot_bridge = robot
        # wait_task = WaitTask(name="waiting", wait_time=2.0)
        # self.tasks.append(wait_task)
        undock_task = UndockTask(name="undock", robot_bridge=self.robot_bridge)
        self.tasks.append(undock_task)
    
    def start(self):
        """ Extends the start() method, to update the mission state in the robot """
        super().start()
        self.robot_bridge.update_mission_state({"mission_state": "START", "mission_state_message": "{} Mission {}: starting".format(self.mission_type, self.name)})

class LoadCargoMission(Mission):
    def __init__(self, name, location, map_name, navigator: BasicNavigator, robot: RobotBridge, map_manager: MapManager, elevator_api: ElevatorAPI):
        super().__init__(name, "LoadCargo")
        self.robot_bridge = robot
        self.map_manager = map_manager
        self.elevator_api = elevator_api

        # Undock if necessary
        if self.robot_bridge.is_docked():
            undock_task = UndockTask(name="undock", robot_bridge=self.robot_bridge)
            self.tasks.append(undock_task)

        # Get navigation waypoints
        waypoints = self.map_manager.get_waypoints(location, map_name)
        for idx, waypoint in enumerate(waypoints):
            self.tasks.append(GoToTask(name="nav2"+waypoint[1], destination=waypoint[1], map_name=waypoint[0], robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager))
            if idx != len(waypoints)-1:
                self.tasks.append(TakeElevatorTask(name="elevator2"+waypoints[idx+1][1], start_floor=waypoint[0], destiny_floor=waypoints[idx+1][0], map_manager=map_manager, elevator_api=elevator_api))

        # wait_task = WaitTask(name="waiting", wait_time=2.0)
        # self.tasks.append(wait_task)
        load_task = LoadTask(name="pickup", robot_bridge=self.robot_bridge)
        self.tasks.append(load_task)

    def start(self):
        """ Extends the start() method, to update the mission state in the robot """
        super().start()
        self.robot_bridge.update_mission_state({"mission_state": "START", "mission_state_message": "{} Mission {}: starting".format(self.mission_type, self.name)})

class DeliverCargoMission(Mission):
    def __init__(self, name, location, map_name, navigator: BasicNavigator, robot: RobotBridge, map_manager: MapManager, elevator_api: ElevatorAPI, trays=["front", "top_tray", "bottom_tray"], autonomous=False):
        super().__init__(name, "DeliverCargo")
        self.location = location
        self.deliver_trays = trays
        self.autonomous = autonomous
        self.robot_bridge = robot
        self.map_manager = map_manager
        self.elevator_api = elevator_api

        # Undock if necessary
        if self.robot_bridge.is_docked():
            undock_task = UndockTask(name="undock", robot_bridge=self.robot_bridge)
            self.tasks.append(undock_task)

        # Get navigation waypoints
        waypoints = self.map_manager.get_waypoints(location, map_name)
        for idx, waypoint in enumerate(waypoints):
            self.tasks.append(GoToTask(name="nav2"+waypoint[1], destination=waypoint[1], map_name=waypoint[0], robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager))
            if idx != len(waypoints)-1:
                self.tasks.append(TakeElevatorTask(name="elevator2"+waypoints[idx+1][0], start_floor=waypoint[0], destiny_floor=waypoints[idx+1][0], map_manager=map_manager, elevator_api=elevator_api))

        # wait_task = WaitTask(name="WaitTask", wait_time=5.0)
        # self.tasks.append(wait_task)
        deliver_task = DeliverTask(name="DeliverCargoTask", robot_bridge=self.robot_bridge, trays=self.deliver_trays, autonomous=self.autonomous)
        self.tasks.append(deliver_task)

    def start(self):
        """ Extends the start() method, to update the mission state in the robot """
        super().start()
        self.robot_bridge.update_mission_state({"mission_state": "START", "mission_state_message": "{} Mission {}: starting".format(self.mission_type, self.name)})

class CUFPickup(Mission):
    def __init__(self, name, location, map_name, navigator: BasicNavigator, robot: RobotBridge, map_manager: MapManager, elevator_api: ElevatorAPI, door_api):
        super().__init__(name, "CUFPickup")
        self.map_name = map_name
        self.location = location
        self.robot_bridge = robot
        self.map_manager = map_manager
        self.elevator_api = elevator_api
        self.door_api = door_api

        # Undocking (even if not docked at the moment)
        # undock_task = UndockTask(name="undock", robot_bridge=self.robot_bridge)
        # self.tasks.append(undock_task)
        
        # goto_pharmacy = GoToTask(name="nav2pharmacy", destination="pharmacy", map_name="floor-1", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager)
        # self.tasks.append(goto_pharmacy)

        load_task = LoadTask(name="pickup", robot_bridge=self.robot_bridge)
        self.tasks.append(load_task)

    def start(self):
        """ Extends the start() method, to update the mission state in the robot """
        super().start()
        self.robot_bridge.update_mission_state({"mission_state": "START", "mission_state_message": "{} Mission {}: starting".format(self.mission_type, self.name)})
        

class CUFDelivery(Mission):
    def __init__(self, name, location, map_name, navigator: BasicNavigator, robot: RobotBridge, map_manager: MapManager, elevator_api: ElevatorAPI, door_api, trays=[]):
        super().__init__(name, "CUFDeliver")
        self.map_name = map_name
        self.location = location
        self.trays = trays
        self.robot_bridge = robot
        self.map_manager = map_manager
        self.elevator_api = elevator_api
        self.door_api = door_api

        # Undocking (even if not docked at the moment)
        undock_task = UndockTask(name="undock", robot_bridge=self.robot_bridge)
        self.tasks.append(undock_task)
        
        # GO TO WITH DOORS AND ELEVATORS
        goto_door1 = GoToTask(name="nav2door1", destination="door1_out", map_name="floor-1", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager)
        self.tasks.append(goto_door1)
        wait_door1 = OpenDoorTask(name="open_door1", door_api=door_api)
        self.tasks.append(wait_door1)
        
        # Trigger Obstruction
        obstruction_task = TriggerObstructionTask(name="hri_obstruction", robot_api=self.robot_bridge)
        self.tasks.append(obstruction_task)
        
        goto_elevator = GoToTask(name="nav2elevator", destination="elevator_in", map_name="floor-1", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager)
        self.tasks.append(goto_elevator)
        elevator_task = TakeElevatorTask(name="take_elevador1", start_floor="floor-1", destiny_floor="floor1", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager, elevator_api=elevator_api)
        self.tasks.append(elevator_task)

    def start(self):
        """ Extends the start() method, to update the mission state in the robot """
        super().start()
        self.robot_bridge.update_mission_state({"mission_state": "START", "mission_state_message": "{} Mission {}: starting".format(self.mission_type, self.name)})

class CUFDelivery2(Mission):
    def __init__(self, name, location, map_name, navigator: BasicNavigator, robot: RobotBridge, map_manager: MapManager, elevator_api: ElevatorAPI, door_api, trays=[]):
        super().__init__(name, "CUFDeliver2")
        self.map_name = map_name
        self.location = location
        self.trays = trays
        self.robot_bridge = robot
        self.map_manager = map_manager
        self.elevator_api = elevator_api
        self.door_api = door_api

        # Undocking (even if not docked at the moment)
        # undock_task = UndockTask(name="undock", robot_bridge=self.robot_bridge)
        # self.tasks.append(undock_task)

        # goto_bump = GoToTask(name="nav2bump", destination="bump_point", map_name="floor1", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager)
        # self.tasks.append(goto_bump)
        # bump_task = TriggerMitigationTask(name="mitigation_task", robot_api=self.robot_bridge)
        # self.tasks.append(bump_task)

        # goto_door_out = GoToTask(name="nav2ucip", destination="ucip", map_name="floor1", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager)
        # self.tasks.append(goto_door_out)
        # deliver_task = DeliverTask(name="ucip_deliver", robot_bridge=self.robot_bridge, trays=["bottom_tray"])
        # self.tasks.append(deliver_task)

        goto_elevator = GoToTask(name="nav2elevator", destination="elevator_in", map_name="floor1", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager)
        self.tasks.append(goto_elevator)
        elevator_task = TakeElevatorTask(name="take_elevador2", start_floor="floor1", destiny_floor="floor5", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager, elevator_api=elevator_api)
        self.tasks.append(elevator_task)
        
class CUFDelivery3(Mission):
    def __init__(self, name, location, map_name, navigator: BasicNavigator, robot: RobotBridge, map_manager: MapManager, elevator_api: ElevatorAPI, door_api, trays=[]):
        super().__init__(name, "CUFDeliver3")
        self.map_name = map_name
        self.location = location
        self.trays = trays
        self.robot_bridge = robot
        self.map_manager = map_manager
        self.elevator_api = elevator_api
        self.door_api = door_api

        # Undocking (even if not docked at the moment)
        # undock_task = UndockTask(name="undock", robot_bridge=self.robot_bridge)
        # self.tasks.append(undock_task)

        # Move to elevator doors
        # goto_door_in = GoToTask(name="nav2door5", destination="door_elev_in", map_name="floor5", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager)
        # self.tasks.append(goto_door_in)
        # wait_door5 = OpenDoorTask(name="open_door5", door_api=door_api)
        # self.tasks.append(wait_door5)
        
        # Move to BalcaoA doors
        # goto_balcao_door_out = GoToTask(name="nav2balcaodoor5", destination="door_balcao_out", map_name="floor5", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager)
        # self.tasks.append(goto_balcao_door_out)
        # wait_door_balcao = OpenDoorTask(name="open_door_balcao", door_api=door_api)
        # self.tasks.append(wait_door_balcao)
        
        goto_balcao = GoToTask(name="nav2balcao", destination="balcao_a", map_name="floor5", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager)
        self.tasks.append(goto_balcao)        
        deliver_balcao_task = DeliverTask(name="balcao_deliver", robot_bridge=self.robot_bridge, trays=["front"])
        self.tasks.append(deliver_balcao_task)

        # Move to BalcaoA doors
        goto_balcao_door_in = GoToTask(name="nav2balcaodoor5_in", destination="door_balcao_in", map_name="floor5", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager)
        self.tasks.append(goto_balcao_door_in)
        # wait_door_balcao_in = OpenDoorTask(name="open_door_balcao_in", door_api=door_api)
        # self.tasks.append(wait_door_balcao_in)

        # Move to elevator doors
        # goto_door_out = GoToTask(name="nav2door5_out", destination="door_elev_out", map_name="floor5", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager)
        # self.tasks.append(goto_door_out)
        # wait_door5 = OpenDoorTask(name="open_door5_out", door_api=door_api)
        # self.tasks.append(wait_door5)
        
        # goto_elevator = GoToTask(name="nav2elevator", destination="elevator_in", map_name="floor5", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager)
        # self.tasks.append(goto_elevator)
        # elevator_task = TakeElevatorTask(name="take_elevador3", start_floor="floor5", destiny_floor="floor-1", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager, elevator_api=elevator_api)
        # self.tasks.append(elevator_task)
    
class CUFArmDelivery(Mission):
    def __init__(self, name, location, map_name, navigator: BasicNavigator, robot: RobotBridge, map_manager: MapManager, elevator_api: ElevatorAPI, door_api, trays=[]):
        super().__init__(name, "CUFArmDelivery")
        self.map_name = map_name
        self.location = location
        self.trays = trays
        self.robot_bridge = robot
        self.map_manager = map_manager
        self.elevator_api = elevator_api
        self.door_api = door_api

        # Undocking (even if not docked at the moment)
        # undock_task = UndockTask(name="undock", robot_bridge=self.robot_bridge)
        # self.tasks.append(undock_task)

        # Move to elevator doors
        # goto_door_in = GoToTask(name="nav2door5", destination="door_elev_in", map_name="floor5", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager)
        # self.tasks.append(goto_door_in)
        # wait_door5 = OpenDoorTask(name="open_door5", door_api=door_api)
        # self.tasks.append(wait_door5)
        
        # # Move to BalcaoA doors
        # goto_balcao_door_out = GoToTask(name="nav2balcaodoor5", destination="door_balcao_out", map_name="floor5", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager)
        # self.tasks.append(goto_balcao_door_out)
        # wait_door_balcao = OpenDoorTask(name="open_door_balcao", door_api=door_api)
        # self.tasks.append(wait_door_balcao)
        
        # goto_balcao = GoToTask(name="nav2balcao", destination="balcao_a_arm", map_name="floor5", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager)
        # self.tasks.append(goto_balcao)        
        deliver_balcao_task = DeliverTask(name="balcao_deliver", robot_bridge=self.robot_bridge, trays=["top_tray"], autonomous=True)
        self.tasks.append(deliver_balcao_task)

        # Move to BalcaoA doors
        # goto_balcao_door_in = GoToTask(name="nav2balcaodoor5_in", destination="door_balcao_in", map_name="floor5", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager)
        # self.tasks.append(goto_balcao_door_in)
        # wait_door_balcao_in = OpenDoorTask(name="open_door_balcao_in", door_api=door_api)
        # self.tasks.append(wait_door_balcao_in)

        # # Move to elevator doors
        # goto_door_out = GoToTask(name="nav2door5_out", destination="door_elev_out", map_name="floor5", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager)
        # self.tasks.append(goto_door_out)
        # wait_door5 = OpenDoorTask(name="open_door5_out", door_api=door_api)
        # self.tasks.append(wait_door5)
        
        # goto_elevator = GoToTask(name="nav2elevator", destination="elevator_in", map_name="floor5", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager)
        # self.tasks.append(goto_elevator)
        # elevator_task = TakeElevatorTask(name="take_elevador3", start_floor="floor5", destiny_floor="floor-1", robot_bridge=self.robot_bridge, navigator=navigator, map_manager=map_manager, elevator_api=elevator_api)
        # self.tasks.append(elevator_task)

class CUFDocking(Mission):
    def __init__(self, name):
        super().__init__(name, "CUFDocking")


if __name__=="__main__":
    rclpy.init()
    
    # m = GoToMission("goto_test", "idmind")
    m = LoadCargoMission("LoadTest", "here", "here", navigator=BasicNavigator())
    m.start()
    while not m.is_finished():
        try:
            m.update()
            print(m)
            time.sleep(0.5)
        except KeyboardInterrupt:
            print("Mission interrupted")
            break
