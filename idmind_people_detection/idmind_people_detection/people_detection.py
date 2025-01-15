#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

import os
import cv2
import copy
from ultralytics import YOLO
from cv_bridge import CvBridge

from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
# from idmind_msgs.msg import Alarm

from geometry_msgs.msg import PoseArray, Pose
from ament_index_python.packages import get_package_share_directory

class PeopleDetectionNode(Node):
    def __init__(self, verbose=2):
        super().__init__('people_detection_node')
        self.verbose = verbose
        self.serv_cb_group = ReentrantCallbackGroup()
        self.pub_cb_group = ReentrantCallbackGroup()
        self.main_cb_group = ReentrantCallbackGroup()

        node_prefix = self.get_name()+"/"
        self.ready = False
        self.ready_srv = self.create_service(Trigger, node_prefix+"ready", self.report_ready, callback_group=self.serv_cb_group)

        #########
        #  ROS  #
        #########
        # Parameters
        self.control_freq       = self.declare_parameter("control_freq", 20.0, ParameterDescriptor(description="Frequency of the main loop")).get_parameter_value().double_value
        self.add_on_set_parameters_callback(self.update_parameters)

        # Subscribers
        self.obj_model = YOLO(os.path.join(get_package_share_directory("idmind_people_detection"), "data", "yolov8n.pt")) 
        self.pose_model = YOLO(os.path.join(get_package_share_directory("idmind_people_detection"), "data", "yolov8n-pose.pt")) 
        # self.obj_model = YOLO(os.path.join(get_package_share_directory("idmind_people_detection"), "data", "yolov8m.pt")) 
        # self.pose_model = YOLO(os.path.join(get_package_share_directory("idmind_people_detection"), "data", "yolov8m-pose.pt")) 
        # self.obj_model = YOLO(os.path.join(get_package_share_directory("idmind_people_detection"), "data", "yolov8x.pt")) 
        # self.pose_model = YOLO(os.path.join(get_package_share_directory("idmind_people_detection"), "data", "yolov8x-pose.pt")) 

        self.cv_bridge = CvBridge()
        self.camera_feed = None
        self.camera_feed_sub = self.create_subscription(Image, "azure_front/rgb/image_raw", self.update_camera_feed, 10, callback_group=self.pub_cb_group)

        # Publishers
        self.annotated_pub = self.create_publisher(Image, node_prefix+"yolo", 10, callback_group=self.pub_cb_group)
        self.people_pose = self.create_publisher(Image, node_prefix+"people_pose", 10, callback_group=self.pub_cb_group)
        self.alarm_active = True
        self.alarm_raised = False
        self.alarm_raised_ts = self.get_clock().now().to_msg()
        self.alarm_pub = self.create_publisher(String, "mission/alarm", 10, callback_group=self.pub_cb_group)
        # self.people_pose_pub = self.create_publisher(PoseArray, "people_pose", 10, callback_group=self.pub_cb_group)

        # Services
        self.enable_alarm_sub = self.create_subscription(Bool, node_prefix+"enable_alarm", self.update_enable_alarm, 10, callback_group=self.pub_cb_group)
        # self.enable_alarm_srv = self.create_service(SetBool, node_prefix+"enable_alarm", self.handle_enable_alarm, callback_group=self.serv_cb_group)
        
        self.ready = True
        self.in_loop = False
        self.main_timer = self.create_timer(1.0/self.control_freq, self.start, callback_group=self.main_cb_group)
        self.log("Node initiated", 2)
        

    #########################
    #  AUXILIARY FUNCTIONS  #
    #########################
    def getDt(self, last):
        current_time = self.get_clock().now().to_msg()

        ct = current_time.sec + (current_time.nanosec/1e+9)
        lt = last.sec + (last.nanosec/1e+9)
        dt = (ct - lt)
        return dt

    def log(self, msg, msg_level, log_level=-1, alert="info"):
        """
        Log function that publish in screen and in topic
        :param msg: Message to be published
        :param msg_level: Message level (1-10, where 1 is most important)
        :param log_level: Message level for logging (1-10, optional, -1 uses the same as msg_level)
        :param alert: Alert level of message - "info", "warn" or "error"
        :return:
        """
        if self.verbose >= msg_level:
            if alert == "info":
                self.get_logger().info(msg)
            elif alert == "warn":
                self.get_logger().warning(msg)
            elif alert == "error":
                self.get_logger().error(msg)

    ###################
    #    CALLBACKS    #
    ###################
    def report_ready(self, _req, response):
        """ Simple Service callback to show node is ready """
        self.get_logger().info("Replying to 'ready' request")
        response.success = self.ready
        response.message = self.get_name()+" is " + ("ready" if self.ready else "not ready")
        return response

    def update_parameters(self, params):
        msg = "\n".join(["{} to {}".format(p.name, p.value) for p in params])
        self.log("Changing parameters: {}" .format(msg), 2)
        for p in params:
            if p.name == "control_freq":
                self.control_freq = p.value
                self.main_timer.timer_period_ns = 1e9/self.control_freq
        return SetParametersResult(successful=True)

    def update_camera_feed(self, msg:Image) -> None:
        self.camera_feed = msg

    def update_enable_alarm(self, msg: Bool) -> None:
        self.alarm_active = msg.data

    # def handle_enable_alarm(self, req:SetBool.Request, res:SetBool.Response) -> SetBool.Response:
    #     self.alarm_active = req.data
    #     res.success = True
    #     res.message = "Alarm is now {}".format("enabled" if req.data else "disabled")
    #     return res

    ####################
    #  Loop Functions  #
    ####################
    def process_poses(self) -> bool:
        if self.camera_feed is None:
            return True

        msg = copy.copy(self.camera_feed)
        img = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        # img = cv2.imread(os.path.join(get_package_share_directory("idmind_people_detection"), "data", "photo_test.jpg"))
        results = self.pose_model.predict(img)
        
        annotated_frame = results[0].plot()
        ratio = 1.0
        # self.log("Shape {}".format(annotated_frame.shape), 1) -> 2048*1536
        # resized_frame = cv2.resize(annotated_frame, (int(annotated_frame.shape[1]*ratio), int(annotated_frame.shape[0]*ratio)))
        img_msg = self.cv_bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        self.people_pose.publish(img_msg)
        # key_names = {"Nose": 0, "Left Eye": 1, "Right Eye": 2, "Left Ear": 3, "Right Ear": 4,
        #              "Left Shoulder": 5, "Right Shoulder": 6, "Left Elbow": 7, "Right Elbow": 8, "Left Wrist": 9, "Right Wrist": 10,
        #              "Left Hip": 11, "Right Hip": 12, "Left Knee": 13, "Right Knee": 14, "Left Ankle": 15, "Right Ankle": 16
        #               }
        key_names = ["Nose", "Left Eye", "Right Eye", "Left Ear", "Right Ear",
                     "Left Shoulder", "Right Shoulder", "Left Elbow", "Right Elbow", "Left Wrist", "Right Wrist",
                     "Left Hip", "Right Hip", "Left Knee", "Right Knee", "Left Ankle", "Right Ankle"]
        
        # alarm_msg = Alarm()
        # alarm_msg.header.stamp = self.get_clock().now().to_msg()
        # alarm_msg.header.frame_id = "azure"
        # alarm_msg.description = "Someone has fallen!"
        # alarm_msg.img = img_msg
        

        # There is only 1 result, because there is only 1 input
        if not self.alarm_active:
            return True

        for r in results:
            # print("L: {} | K: {}".format(len(r.keypoints), r.keypoints))
            # r.keypoints has one element per skeleton detected
            print("=== Result ===")
            # r.boxes has one element per skeleton detected
            boxes = r.boxes
            print("Box Nrs: {}".format(len(boxes)))

            for box_idx, box in enumerate(boxes):
                if box.conf > 0.5:
                    # class_name = self.pose_model.names[int(box.cls)]
                    # print("{} - {}".format(class_name, box))
                    print("Box Shape: {}".format(box.xywh.shape))
                    [x, y, w, h] = box.xywh[0].to('cpu').detach().numpy().copy()
                    if h > w:
                        self.log("Person is standing", 1)
                    else:
                        self.log("Person may be laying - {}:{}, {}:{}".format(int(x), int(x+w), int(y), int(y+h)), 1)
                        # r.keypoints has one element per skeleton detected
                        ks = r.keypoints.cpu().numpy()
                        # I hope these match...
                        k = ks[box_idx]
                        print("=== Keypoint ===")
                        kp = k.data[0]
                        if len(kp) > 0:
                            # How many points are valid?
                            upper_visible = any([kp[keypart][0]!=0 or kp[keypart][1] != 0 for keypart in range(0, 5)])
                            middle_visible = any([kp[keypart][0]!=0 or kp[keypart][1] != 0 for keypart in range(5, 13)])
                            lower_visible = any([kp[keypart][0]!=0 or kp[keypart][1] != 0 for keypart in range(13, 16)])

                            if upper_visible and middle_visible and not lower_visible:
                                print("Person is probably sitting")
                            elif upper_visible and middle_visible and lower_visible:
                                try:
                                    print("Person is probably laying")
                                    if self.getDt(self.alarm_raised_ts) > 1.0:
                                        self.alarm_raised = False
                                    if not self.alarm_raised:
                                        [x1, y1, x2, y2] = box.xyxy[0].to('cpu').detach().numpy().copy()
                                        fallen_box = img[int(y1):int(y2), int(x1):int(x2), :]
                                        [dx, dy, dz] = fallen_box.shape
                                        goal_width = 250
                                        fallen_box_res = cv2.resize(fallen_box, (int(goal_width*dy/dx), goal_width))
                                        path = "/home/idmind-harmonytop/hmi/hmi_harmony_ros2/src/idmind_people_detection/webserver/img/alarm.jpg"
                                        if cv2.imwrite(path, fallen_box_res):
                                            self.log("Image saved", 1, alert="warn")
                                        else:
                                            self.log("Image failed", 1, alert="error")
                                        # img_msg = self.cv_bridge.cv2_to_imgmsg(fallen_box, encoding='bgr8')
                                        self.alarm_raised = True
                                        self.alarm_raised_ts = self.get_clock().now().to_msg()
                                        alarm_msg = String(data="alarm.jpg")
                                        
                                        self.alarm_pub.publish(alarm_msg)
                                except Exception as err:
                                    self.log("Exception in getting fallen box: {}".format(err), 1, alert="error")
                            # for idx, name in enumerate(key_names):
                            #     print("{} at [{}, {}] with {} conf".format(name, kp[idx][0], kp[idx][1], kp[idx][2]))

        return True

    def process_image(self) -> bool:
        
        if self.camera_feed is None:
            return True

        msg = copy.copy(self.camera_feed)
        img = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.obj_model.predict(img) 

        people_array = PoseArray()
        people_array.header.stamp = self.get_clock().now().to_msg()
        people_array.header.frame_id
        
        # There is only 1 result... maybe because only 1 image?
        for r in results:
            # Result has attributes: boxes, orig_shape path, probs, save_dir, speed
            boxes = r.boxes
            for box in boxes:
                # Box has attributes cls, conf, data, id, is_track, orig_shape, shape, xywh, xywhn, xyxy, xyxyn
                # self.get_logger().info("Box: {}".format(box))
                class_name = self.obj_model.names[int(box.cls)]
                if class_name == "person":
                    box_coord = box.xyxy[0].to('cpu').detach().numpy().copy()
                    self.get_logger().info("We have a person from [{}, {}] to [{}, {}] of {}".format(box_coord[1], box_coord[0], box_coord[3], box_coord[2], img.shape))
                    
                    # Crop image of a person
                    crop_img = img[int(box_coord[1]):int(box_coord[3]), int(box_coord[0]):int(box_coord[2])]
                    img_msg2 = self.cv_bridge.cv2_to_imgmsg(crop_img, encoding='bgr8')
                    self.people_pose.publish(img_msg2)

        annotated_frame = results[0].plot()
        img_msg = self.cv_bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        self.annotated_pub.publish(img_msg)
        
        return True

    def shutdown(self):
        """
        Shutdown rotine.
        :return:
        """
        return True

    def start(self):
        """ Main method to be called periodically """
        if self.in_loop:
            return True
        else:
            self.in_loop = True

        if rclpy.ok():
            try:
                # self.process_image()
                self.process_poses()
            except KeyboardInterrupt:
                self.shutdown()
                self.main_timer.cancel()
            # except Exception as err:
            #     self.log("Exception caught in main: {}".format(err), 1, alert="error")

        self.in_loop = False

def main(args=None):
    rclpy.init(args=args)
    
    people_detection_node = PeopleDetectionNode()    
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(people_detection_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        print('\033[91m'+"Shutting down PeopleDetection Node"+'\033[0m')
    finally:
        executor.shutdown()
        people_detection_node.shutdown()
        people_detection_node.destroy_node()
        
    try:
        rclpy.shutdown()
    except Exception:
        pass

if __name__ == '__main__':
    main()
