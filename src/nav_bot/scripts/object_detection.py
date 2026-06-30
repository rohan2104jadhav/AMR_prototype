#!/usr/bin/env python3
"""
object_pursuit_node.py

Detects a target object class with YOLOv8, estimates its 3D position using
an aligned depth image + camera intrinsics, transforms that position into
the map frame, and sends it to Nav2 as a NavigateToPose goal.

Obstacle avoidance is NOT implemented here - that's Nav2's job, driven by
whatever populates your local costmap (lidar scan or depth pointcloud).
This node only decides WHERE the robot should go.
"""

import math
import os

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import tf2_ros
from tf2_geometry_msgs import do_transform_point

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from nav2_msgs.action import NavigateToPose

from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2

import message_filters


# Don't send a new Nav2 goal unless the target moved at least this far (m)
GOAL_UPDATE_DISTANCE_THRESHOLD = 0.3

# Minimum seconds between goal updates, even if the target moved
GOAL_UPDATE_MIN_INTERVAL = 1.5

# Stop this far short of the object's center instead of driving into it
GOAL_STANDOFF_DISTANCE = 0.6


class ObjectPursuitNode(Node):
    def __init__(self):
        super().__init__('object_pursuit_node')

        # --- Dynamic model / target selection ---
        # Default: stock COCO-pretrained model, no dataset/training needed.
        # Switch to your custom model later with launch-time params, e.g.:
        #   ros2 run nav_bot object_pursuit_node --ros-args \
        #     -p model_path:=$HOME/yolo_dataset/runs/detect/train/weights/best.pt \
        #     -p target_class:=cube
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('target_class', 'chair')

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.target_class = self.get_parameter('target_class').get_parameter_value().string_value

        self.bridge = CvBridge()
        # 'yolov8n.pt' (no slash, no .pt path on disk) auto-downloads the stock
        # COCO model. A real path like '~/yolo_dataset/.../best.pt' loads your
        # custom-trained weights instead - same code, just a different param.
        resolved_path = os.path.expanduser(model_path)
        self.get_logger().info(
            f'Loading model: {resolved_path} | target class: "{self.target_class}"')
        self.model = YOLO(resolved_path)

        self.camera_intrinsics = None
        self.create_subscription(
            CameraInfo, '/depth_camera/camera_info',
            self.camera_info_callback, qos_profile_sensor_data)

        # Synchronize color + depth frames so they correspond to the same instant
        color_sub = message_filters.Subscriber(
            self, Image, '/depth_camera/image', qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(
            self, Image, '/depth_camera/depth_image',
            qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=5, slop=0.05)
        self.sync.registerCallback(self.image_callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publishes the camera frame with detection boxes drawn on it,
        # so you can actually see what YOLO is finding in rqt_image_view.
        self.debug_image_pub = self.create_publisher(Image, '/yolo/detection_image', 10)

        self.last_goal_position = None
        self.last_goal_time = self.get_clock().now()
        self.current_goal_handle = None

        self.get_logger().info('Object pursuit node started, waiting for camera info...')

    def camera_info_callback(self, msg: CameraInfo):
        # fx, fy, cx, cy from the intrinsic matrix
        self.camera_intrinsics = {
            'fx': msg.k[0], 'fy': msg.k[4],
            'cx': msg.k[2], 'cy': msg.k[5],
        }

    def image_callback(self, color_msg: Image, depth_msg: Image):
        if self.camera_intrinsics is None:
            return 

        frame = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
    
        results = self.model(frame, verbose=False)[0]

        best_box = None
        best_conf = 0.0
        for box in results.boxes:
            cls_name = self.model.names[int(box.cls[0])]
            conf = float(box.conf[0])
            bx1, by1, bx2, by2 = [int(v) for v in box.xyxy[0].tolist()]

            is_target = (cls_name == self.target_class)
            color = (0, 255, 0) if is_target else (0, 165, 255)  # green for target, orange for others
            cv2.rectangle(frame, (bx1, by1), (bx2, by2), color, 2)
            cv2.putText(frame, f'{cls_name} {conf:.2f}', (bx1, max(by1 - 8, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            if is_target and conf > best_conf:
                best_conf = conf
                best_box = box

        # Publish the annotated frame regardless of whether the target was
        # found - this is what you point rqt_image_view at to actually see
        # what YOLO is detecting (or failing to detect) in real time.
        annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        annotated_msg.header = color_msg.header
        self.debug_image_pub.publish(annotated_msg)

        if best_box is None:
            return  # nothing matching the target class this frame

        x1, y1, x2, y2 = best_box.xyxy[0].tolist()
        cx_px = int((x1 + x2) / 2)
        cy_px = int((y1 + y2) / 2)

        # Gazebo's rgbd_camera publishes depth as 32FC1 - float meters directly,
        # no mm conversion needed (unlike real RealSense hardware, which uses
        # 16-bit mm integers).
        z = float(depth[cy_px, cx_px])
        if z <= 0.0 or math.isnan(z) or math.isinf(z):
            self.get_logger().warn('Invalid depth at detection center, skipping frame')
            return

        # Pixel -> camera-frame 3D point (pinhole projection inverse)
        fx, fy = self.camera_intrinsics['fx'], self.camera_intrinsics['fy']
        cx0, cy0 = self.camera_intrinsics['cx'], self.camera_intrinsics['cy']
        x = (cx_px - cx0) * z / fx
        y = (cy_px - cy0) * z / fy

        camera_point = PointStamped()
        camera_point.header = color_msg.header
        camera_point.point.x = x
        camera_point.point.y = y
        camera_point.point.z = z

        try:
            # rclpy.time.Time() with no args means "latest available transform",
            # not "the literal time 0" - this avoids extrapolation errors as
            # long as TF is actually publishing recently. If you still see
            # extrapolation warnings, the real fix is making sure every node
            # (this one, robot_state_publisher, slam_toolbox) is launched with
            # use_sim_time:=true so they all agree on what "now" means.
            transform = self.tf_buffer.lookup_transform(
                'map', color_msg.header.frame_id, rclpy.time.Time())
            map_point = do_transform_point(camera_point, transform)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return

        map_pose = PoseStamped()
        map_pose.header.frame_id = 'map'
        map_pose.header.stamp = color_msg.header.stamp
        map_pose.pose.position = map_point.point
        map_pose.pose.orientation.w = 1.0

        # Pull the goal back to a standoff distance short of the object's
        # actual center, using the robot's current position in map frame.
        # Without this, the goal sits AT the object - which is often inside
        # the inflated costmap radius and unreachable, especially if the
        # object is near a wall or another obstacle.
        map_pose = self.apply_standoff(map_pose)
        if map_pose is None:
            return

        self.maybe_send_goal(map_pose, color_msg.header)

    def apply_standoff(self, map_pose: PoseStamped):
        try:
            robot_transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            self.get_logger().warn(f'Could not get robot pose for standoff calc: {e}')
            return None

        rx = robot_transform.transform.translation.x
        ry = robot_transform.transform.translation.y
        ox = map_pose.pose.position.x
        oy = map_pose.pose.position.y

        dx, dy = ox - rx, oy - ry
        dist = math.hypot(dx, dy)
        if dist < GOAL_STANDOFF_DISTANCE:
            return None  # already closer than the standoff distance, nothing to send

        scale = (dist - GOAL_STANDOFF_DISTANCE) / dist
        map_pose.pose.position.x = rx + dx * scale
        map_pose.pose.position.y = ry + dy * scale
        return map_pose

    def maybe_send_goal(self, map_pose: PoseStamped, header):
        now = self.get_clock().now()
        seconds_since_last = (now - self.last_goal_time).nanoseconds / 1e9

        if self.last_goal_position is not None:
            dx = map_pose.pose.position.x - self.last_goal_position[0]
            dy = map_pose.pose.position.y - self.last_goal_position[1]
            moved = math.hypot(dx, dy)
            if moved < GOAL_UPDATE_DISTANCE_THRESHOLD and \
                    seconds_since_last < GOAL_UPDATE_MIN_INTERVAL:
                return  # target hasn't moved meaningfully, don't spam Nav2

        if not self.nav_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('Nav2 action server not available yet')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose = map_pose.pose
        goal_msg.pose.pose.orientation.w = 1.0  # ignore depth-derived rotation noise

        # Cancel any in-flight goal before sending the updated one
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

        self.last_goal_position = (map_pose.pose.position.x, map_pose.pose.position.y)
        self.last_goal_time = now
        self.get_logger().info(
            f'Sent new goal: x={map_pose.pose.position.x:.2f}, '
            f'y={map_pose.pose.position.y:.2f}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            return
        self.current_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        status = future.result().status
        # GoalStatus: 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
        if status == 4:
            self.get_logger().info('Reached goal successfully')
        elif status == 5:
            pass  # canceled by us, because the target moved - expected, not an error
        else:
            self.get_logger().warn(
                f'Nav2 goal failed (status={status}) - likely blocked by an '
                'obstacle. Forcing a resend on the next detection.')
            # Clear the cooldown so maybe_send_goal doesn't suppress the retry
            # just because the target object hasn't moved since the failure.
            self.last_goal_position = None


def main(args=None):
    rclpy.init(args=args)
    node = ObjectPursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()