#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.client import Client 

from idmind_msgs.msg import LockStatus
from idmind_msgs.srv import SetString
from std_msgs.msg import Bool, String, ColorRGBA
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Vector3
from nav2_msgs.srv import LoadMap, ClearEntireCostmap
from visualization_msgs.msg import MarkerArray, Marker

import tf2_ros
from squaternion import Quaternion
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion

import os
import json
import copy
import numpy as np


class MapManager(Node):
    """ This class will handle Locations (consider extenting for handling map loading and consider extending to independent ROS package/node) """
    def __init__(self, node_name='map_manager', verbose=2):
        super().__init__(node_name=node_name)
        self.verbose = verbose
        self.serv_cb_group = ReentrantCallbackGroup()
        self.sub_cb_group = ReentrantCallbackGroup()
        self.pub_cb_group = ReentrantCallbackGroup()
        self.main_cb_group = ReentrantCallbackGroup()

        node_prefix = self.get_name()+"/"
        self.ready = False
        # self.ready_srv = self.create_service(Trigger, node_prefix+"ready", self.report_ready, callback_group=self.serv_cb_group)

        # Parameters
        def_map_dir=os.path.join(get_package_share_directory("idmind_harmony"), "maps")
        self.map_dir            = self.declare_parameter("map_dir",  def_map_dir, ParameterDescriptor(description="Directory with maps")).get_parameter_value().string_value
        self.map_root           = self.declare_parameter("map_root",  "vodafone", ParameterDescriptor(description="Name of map network root")).get_parameter_value().string_value
        self.goal_map_name      = self.declare_parameter("map_name",  "bc", ParameterDescriptor(description="Name of the current map")).get_parameter_value().string_value
        self.map_name = None
        self.add_on_set_parameters_callback(self.update_parameters)

        self.load_map_cli = self.create_client(LoadMap, '/harmony/map_server/load_map', callback_group=self.serv_cb_group)
        self.load_map_obs_cli = self.create_client(LoadMap, '/harmony/map_obs_server/load_map', callback_group=self.serv_cb_group)
        self.set_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/harmony/initialpose", 10, callback_group=self.pub_cb_group)
        
        self.amcl_pose = None
        self.amcl_pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/harmony/amcl_pose", self.update_amcl_pose, 10, callback_group=self.sub_cb_group)
        
        #create tf listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        # self.map_root = map_root
        # self.map_name = map_name
        self.locations = {}
        self.locations_pub = self.create_publisher(MarkerArray, node_prefix+"locations", 10, callback_group=self.pub_cb_group)
        self.load_locations()
        self.map_loading = False
        
        self.get_locations_list_srv = self.create_service(Trigger, node_prefix+"get_locations_list", self.handle_get_locations_list, callback_group=self.serv_cb_group)
        self.get_location_srv = self.create_service(SetString, node_prefix+"get_location", self.handle_get_location, callback_group=self.serv_cb_group)
        self.load_map_srv = self.create_service(SetString, node_prefix+"load_map", self.handle_load_map, callback_group=self.serv_cb_group)
        self.save_location_srv = self.create_service(SetString, node_prefix+"save_location", self.handle_save_location, callback_group=self.serv_cb_group)
        
        self.map_timer = self.create_timer(timer_period_sec=2.0, callback=self.map_timer_callback, callback_group=self.serv_cb_group)
        self.get_logger().info("Node is ready")

    #################
    #   CALLBACKS   #
    #################
    def report_ready(self, _req, response:Trigger.Response):
        """ Simple Service callback to show node is ready """
        self.get_logger().info("Replying to 'ready' request")
        response.success = self.ready
        response.message = self.get_name()+" is " + ("ready" if self.ready else "not ready")
        return response

    def update_parameters(self, params):
        msg = "\n".join(["{} to {}".format(p.name, p.value) for p in params])
        self.log("Changing parameters: {}" .format(msg), 2)
        # for p in params:
        #     if p.name == "map_root":
        #         self.map_root = p.value
        #         self.load_map()
        #         self.load_locations(self.map_root)
        #     if p.name == "map_name":
        #         self.goal_map_name = p.value
        #         self.load_map()
        return SetParametersResult(successful=True)

    def update_amcl_pose(self, msg:PoseWithCovarianceStamped):
        self.amcl_pose = msg

    def map_timer_callback(self):
        if not self.map_loading:
            if self.map_name != self.goal_map_name:
                self.get_logger().error("Timer: change map to {}".format(self.goal_map_name))
                if not self.load_map(self.goal_map_name):
                    self.get_logger().warn("Failed to load {} map".format(self.goal_map_name))
    
    def handle_get_locations_list(self, _req, res:Trigger.Response) -> Trigger.Response:
        """ Returns a dictionary with a list of locations per map """
        res.success = True
        res.message = json.dumps(self.get_location_list())
        return res

    def handle_get_location(self, req:SetString.Request, res:SetString.Response) -> SetString.Response:
        """ Returns a JSON dictionary with the location coordinates if successfull """
        d = dict(json.loads(req.data))
        if "location_name" not in d.keys():
            msg = "Dictionary must contain 'location_name' and may contain 'map_name'."
            self.get_logger().error(msg)
            res.success = False
            res.message = msg
            return res
        
        location_name = d["location_name"]
        map_name = None if "map_name" not in d.keys() else d["map_name"]
        loc = self.get_location(location_name, map_name)
        if loc is None:
            msg = location_name+" location was not found in "+("any map." if map_name is None else " the given map")
            self.get_logger().error(msg)
            res.success = False
            res.message = msg
        else:
            res.success = True
            res.message = json.dumps(loc)
        return res

    def handle_load_map(self, req, res):
        """ Returns True if the map was successfully loaded, False otherwise """
        self.goal_map_name = req.data
        start = self.get_clock().now().to_msg()
        while self.calc_dt(start, self.get_clock().now().to_msg()) < 20.0:
            if self.map_name == self.goal_map_name:
                res.success = True
                res.message = "{} map successfully loaded.".format(self.map_name)
                return res
            else:
                self.get_clock().sleep_for(Duration(seconds=1))
            
        res.success = False
        res.message = "Timeout loading {} map.".format(self.goal_map_name)
        return res

    def handle_save_location(self, req:SetString.Request, res:SetString.Response):
        loc_name = req.data
        p = self.get_pose(robot_frame="harmony/base_link", fixed_frame="map")
        x = p.pose.pose.position.x
        y = p.pose.pose.position.y
        q = p.pose.pose.orientation
        th = euler_from_quaternion([q.w, q.x, q.y, q.z])[2]
        
        loc_text = "    "+loc_name+":\n      x: {:.4f}\n      y: {:.4f}\n      th: {:.4f}\n".format(x, y, th)
        fname = os.path.join(self.map_dir, self.map_root+"_"+self.map_name+"_locations.yaml")
        fname = fname.replace("install/idmind_harmony/share", "src")

        print(fname)
        # Assume it already exists,, just append the new point
        with open(fname, "a") as f:
            f.write(loc_text)

        res.success = True
        res.message = "Location saved"
        return res
    
    #################
    #   AUXILIARY   #
    #################
    def calc_dt(self, start, stop):
        """ Computes the diffence in seconds between two timestamps """
        dt = stop.sec + stop.nanosec*1e-9 - start.sec - start.nanosec*1e-9
        return dt
    
    def wait_for_server(self, client:Client, timeout=5.0) -> bool:
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

    ###################
    #   GET METHODS   #
    ###################
    def get_map(self):
        return self.map_name

    def get_location_list(self):
        """ Return the full dictionary of locations stored """
        res = {}
        for map, locations in self.locations.items():
            res[map] = []
            for location in locations.keys():
                res[map].append(location)
        return res


    def get_location(self, location_name, map_name=None):
        """ 
            Return a dictionary with the map root name, map name and PoseStamped of the location in the map
            It will return the first match, if not map_name is provided.
        """
        self.get_logger().debug("Get location of {} in {}".format(location_name, map_name))
        if map_name is not None:
            if map_name not in self.locations.keys():
                self.get_logger().error("Map {} does not exist in the current setting".format(map_name))
                return None
            elif location_name not in self.locations[map_name].keys():
                self.get_logger().error("Location {} does not exist in the given map {}".format(location_name, map_name))
                return None
            else:
                loc = self.locations[map_name][location_name]
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "map"
                pose.pose.position.x = loc["x"]
                pose.pose.position.y = loc["y"]
                th = loc["th"]
                q = Quaternion.from_euler(0.0, 0.0, th, degrees=False)
                pose.pose.orientation.x = q[1]
                pose.pose.orientation.y = q[2]
                pose.pose.orientation.z = q[3]
                pose.pose.orientation.w = q[0]
                
                res = {"map_root": self.map_root, "map_name": map_name, "location_name": location_name}
                res.update({"pose": pose})
        else:   
            res = None
            for map in self.locations.keys():
                if location_name in self.locations[map].keys():
                    loc = self.locations[map][location_name]
                    pose = PoseStamped()
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.header.frame_id = "map"
                    pose.pose.position.x = loc["x"]
                    pose.pose.position.y = loc["y"]
                    th = loc["th"]
                    q = Quaternion.from_euler(0.0, 0.0, th, degrees=False)
                    pose.pose.orientation.x = q[1]
                    pose.pose.orientation.y = q[2]
                    pose.pose.orientation.z = q[3]
                    pose.pose.orientation.w = q[0]
                    
                    res = {"map_root": self.map_root, "map_name": map_name, "location_name": location_name}
                    res.update({"pose": pose})
            if res is None:
                self.get_logger().error("Location {} does not exist in any map".format(location_name))
        
        return res

    def get_pose(self, robot_frame="base_link", fixed_frame="map"):
        
        if self.amcl_pose is None or self.amcl_pose.header.frame_id != fixed_frame:
            t = self.tfBuffer.lookup_transform(robot_frame, fixed_frame, Duration(seconds=0))
            p = PoseWithCovarianceStamped()
            p.header.stamp = t.header.stamp
            p.header.frame_id = t.header.frame_id
            p.pose.pose.position.x = t.transform.translation.x
            p.pose.pose.position.y = t.transform.translation.y
            p.pose.pose.position.z = t.transform.translation.z
            p.pose.pose.orientation = t.transform.rotation
            p.pose.covariance = 0.1*np.identity(6).flatten()
            return p
        else:
            return self.amcl_pose

    def get_waypoints(self, destination, map_name):
        """ Returns a list of waypoints connecting the maps necessary for navigation (or a single waypoint if already in the right map)"""
        if self.map_name == map_name:
            return [[map_name, destination]]
        else:
            return [[self.map_name, "elevator"], [map_name, destination]]
        
    ###################
    #   SET METHODS   #
    ###################
    def publish_locations(self) -> bool:
        """ Publishes current map locations for RViz visualization """
        if self.map_name is not None:
            arr = MarkerArray()
            m = Marker()
            m.action = Marker.DELETEALL
            arr.markers.append(m)
            self.locations_pub.publish(arr)
            locs = self.locations[self.map_name]
            locations_array = MarkerArray()
            curr_time = self.get_clock().now().to_msg()
            loc_id = 0
            for loc, val in locs.items():
                m = Marker()
                m.header.stamp = curr_time
                m.header.frame_id = "map"
                m.ns = self.map_root+"_"+self.map_name+"_"
                m.id = loc_id
                m.type = Marker.ARROW
                m.action = Marker.ADD
                p = Pose()
                p.position.x = val["x"]
                p.position.y = val["y"]
                q = Quaternion.from_euler(0.0, 0.0, val["th"], degrees=False)
                p.orientation.x = q[1]
                p.orientation.y = q[2]
                p.orientation.z = q[3]
                p.orientation.w = q[0]
                m.pose = p
                m.scale = Vector3(x=0.7, y=0.2, z=0.2)
                m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
                locations_array.markers.append(m)
                t = Marker()
                t.header.stamp = curr_time
                t.header.frame_id = "map"
                t.ns = self.map_root+"_"+self.map_name+"_"
                t.id = loc_id + 1
                t.type = Marker.TEXT_VIEW_FACING
                t.action = Marker.ADD
                p2 = copy.deepcopy(p)
                p2.position.x = p2.position.x + 0.5*np.sin(val["th"])
                p2.position.y = p2.position.y - 0.5*np.cos(val["th"])
                t.pose = p2
                t.scale = Vector3(x=0.7, y=0.7, z=0.7)
                t.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
                t.text = loc
                locations_array.markers.append(t)
                loc_id = loc_id+2
            self.locations_pub.publish(locations_array)
        return True

    def load_locations(self) -> bool:
        self.get_logger().info("Loading locations of {} from {}".format(self.map_root, self.map_dir))
        map_network = {}
        # Check all maps of the network
        for fname in os.listdir(self.map_dir):
            if fname.startswith(self.map_root):
                name = fname.split(".")[0].split("_")[1]
                if name not in map_network.keys():
                    map_network[name] = {}
                    self.get_logger().info("Found map {} in {}".format(name, self.map_root))

        # Load all locations from the yaml files
        for map in map_network.keys():
            try:
                with open(os.path.join(self.map_dir, self.map_root+"_"+map+"_locations.yaml"), "r") as fname:
                    d = fname.readlines()
            except FileNotFoundError:
                self.get_logger().error("{} locations file in {} does not exist".format(map, self.map_root))
                continue
            
            # 1st line should have root name
            if d[0].rstrip(":\n") != self.map_root:
                self.get_logger().error("Locations file references {} as root instead of {}".format(d[0].strip(":\n"), self.map_root))
                continue
            # 2nd line should have map name
            elif d[1].lstrip().rstrip(":\n") != map:
                self.get_logger().error("Locations file references {} as submap instead of {}".format(d[1].lstrip().strip(":\n"), map))
                continue
            else:
                # Load locations
                idx = 2
                while idx < len(d):
                    try:
                        if "#" not in d[idx]:
                            name = d[idx].lstrip().strip(":\n")
                            map_network[map][name] = {}
                            for iter in range(1, 4):
                                [coord, val] = d[idx+iter].lstrip().strip("\n").split(":")
                                map_network[map][name][coord] = float(val)
                            self.get_logger().debug("Loaded {} from {}".format(name, map))
                    except Exception as err:
                        self.get_logger().error("Failed to load pose: {}".format(err))
                        continue
                    finally:
                        idx = idx+4
        
        self.locations = map_network
        # print(self.locations)

        return True
    
    def set_goal_map(self, map_name):
        self.goal_map_name = map_name
        start = self.get_clock().now().to_msg()
        while self.map_name != map_name:
            self.get_clock().sleep_for(Duration(seconds=1))
            if self.calc_dt(start, self.get_clock().now().to_msg()) > 10.0:
                return False
        return True
        
    def load_map(self, map_name):
        """
            Loads the map and obstacles map to the map server and updates current location
        """
        if self.map_loading:
            return True
        else:
            self.map_loading = True
        # Check if map exists
        if map_name not in self.locations.keys():
            self.get_logger().warn("{} does not exist in the network. Choices are {}".format(map_name, self.locations.keys()))
            self.map_loading = False
            return False

        # Wait for services
        found_map_server = False
        found_map_obs_server = False
        while not found_map_server or not found_map_obs_server:
            if not found_map_server:
                found_map_server = self.wait_for_server(self.load_map_cli)
            if not found_map_obs_server:
                found_map_obs_server = self.wait_for_server(self.load_map_obs_cli)
            self.get_logger().warn("Waiting for map_server and map_obs_server")
            
        # Load map
        req = LoadMap.Request()
        req.map_url = os.path.join(self.map_dir, self.map_root+"_"+map_name+".yaml")
        map_future = self.load_map_cli.call_async(req)
        req_obs = LoadMap.Request()
        req_obs.map_url = os.path.join(self.map_dir, self.map_root+"_"+map_name+"_obs.yaml")
        map_obs_future = self.load_map_obs_cli.call_async(req_obs)
        
        while not map_future.done():
            self.get_logger().info("Waiting for map loading")
            self.get_clock().sleep_for(Duration(seconds=1))

        while not map_obs_future.done():
            self.get_logger().info("Waiting for map_obs loading")
            self.get_clock().sleep_for(Duration(seconds=1))
        
        res = map_future.result()
        map_success = res.result == res.RESULT_SUCCESS
        res_obs = map_future.result()
        map_obs_success = res_obs.result == res_obs.RESULT_SUCCESS
        if map_success and map_obs_success:
            self.map_name = map_name
            self.goal_map_name = map_name
            self.get_logger().info("{} map loaded.".format(self.map_name))
            self.map_loading = False
            self.publish_locations()
            return True
        else:
            self.get_logger().warn("Failed to load {} map (code {} and {})".format(map_name, res.result, res_obs.result))
            self.map_loading = False
            return False
    
    def set_pose(self, pose:PoseWithCovarianceStamped|PoseStamped):
        """ Relocates the robot using AMCL """
        # Check validity?
        if type(pose) == PoseStamped:
            p = PoseWithCovarianceStamped()
            p.header = pose.header
            p.pose.pose = pose.pose
            p.pose.covariance = 0.1*np.identity(6).flatten()
            self.set_pose_pub.publish(p)
        elif type(pose) == PoseWithCovarianceStamped:
            self.set_pose_pub.publish(pose)
        return True


def main(args=None):
    rclpy.init(args=args)
    idx = 0

    try:
        map_manager = MapManager()
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(map_manager)
        while True:
            try:
                idx += 1
                executor.spin_once()
            except KeyboardInterrupt:
                print('\033[91m'+"Shutting down Map Manager"+'\033[0m')
                break
            
        executor.shutdown()
        map_manager.destroy_node()
    
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
