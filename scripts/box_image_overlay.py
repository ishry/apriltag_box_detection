#!/usr/bin/env python3

import math
import os
import sys
import threading
from collections import OrderedDict

import cv2
import rospy
import rospkg
import yaml
from apriltag_ros.msg import AprilTagDetectionArray

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from box_pose_estimator import BoxPoseEstimator
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image


BOX_EDGES = [
    ((0, 1), (0, 0, 255)), ((3, 2), (0, 0, 255)), ((4, 5), (0, 0, 255)), ((7, 6), (0, 0, 255)),
    ((0, 3), (0, 255, 0)), ((1, 2), (0, 255, 0)), ((4, 7), (0, 255, 0)), ((5, 6), (0, 255, 0)),
    ((0, 4), (255, 0, 0)), ((1, 5), (255, 0, 0)), ((2, 6), (255, 0, 0)), ((3, 7), (255, 0, 0)),
]

BOX_AXES = [
    ("+X", (1.0, 0.0, 0.0), (0, 0, 255)),
    ("+Y", (0.0, 1.0, 0.0), (0, 255, 0)),
    ("+Z", (0.0, 0.0, 1.0), (255, 0, 0)),
]


def quaternion_to_matrix(q):
    x, y, z, w = q
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def pose_to_transform(pose):
    position = (
        pose.position.x,
        pose.position.y,
        pose.position.z,
    )
    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    rotation = quaternion_to_matrix(quaternion)
    return [
        [rotation[0][0], rotation[0][1], rotation[0][2], position[0]],
        [rotation[1][0], rotation[1][1], rotation[1][2], position[1]],
        [rotation[2][0], rotation[2][1], rotation[2][2], position[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def transform_point(t, point):
    return (
        t[0][0] * point[0] + t[0][1] * point[1] + t[0][2] * point[2] + t[0][3],
        t[1][0] * point[0] + t[1][1] * point[1] + t[1][2] * point[2] + t[1][3],
        t[2][0] * point[0] + t[2][1] * point[1] + t[2][2] * point[2] + t[2][3],
    )


def load_config(path):
    with open(path, "r") as config_file:
        config = yaml.safe_load(config_file)
    if not config:
        raise ValueError("empty config file: %s" % path)
    return config


def default_config_path():
    package_path = rospkg.RosPack().get_path("apriltag_box_detection")
    return os.path.join(package_path, "config", "box.yaml")


def default_robot_config_path():
    package_path = rospkg.RosPack().get_path("apriltag_box_detection")
    return os.path.join(package_path, "config", "robot.yaml")


def default_tag_config_path():
    package_path = rospkg.RosPack().get_path("apriltag_box_detection")
    return os.path.join(package_path, "config", "tags.yaml")


class BoxDetectionNode:
    def __init__(self):
        config_file = rospy.get_param("~config_file", default_config_path())
        robot_config_file = rospy.get_param("~robot_config_file", default_robot_config_path())
        tag_config_file = rospy.get_param("~tag_config_file", default_tag_config_path())
        self.config = load_config(config_file)
        self.robot_config = load_config(robot_config_file)
        self.tag_sizes = self.load_tag_sizes(tag_config_file)
        self.robot_tag_ids = self.build_robot_tag_ids(self.robot_config)
        self.sync_queue_size = max(1, int(rospy.get_param("~sync_queue_size", 15)))
        self.input_queue_size = max(1, int(rospy.get_param("~queue_size", 10)))
        self.overlay_enabled = bool(rospy.get_param("~overlay", True))
        self.bridge = CvBridge()
        self.pose_estimator = BoxPoseEstimator(subscribe=False)
        self.warn_overlapping_tag_ids()
        self.camera_info = None
        self.sync_lock = threading.Lock()
        self.render_lock = threading.Lock()
        self.images = OrderedDict()
        self.detection_results = OrderedDict()

        self.image_pub = None
        self.info_sub = None
        self.image_sub = None
        self.tag_sub = rospy.Subscriber(
            "tag_detections",
            AprilTagDetectionArray,
            self.on_tag_detections,
            queue_size=self.input_queue_size,
        )
        if self.overlay_enabled:
            self.image_pub = rospy.Publisher("box_detection_image", Image, queue_size=1)
            self.info_sub = rospy.Subscriber(
                "camera_info",
                CameraInfo,
                self.on_camera_info,
                queue_size=1,
            )
            self.image_sub = rospy.Subscriber(
                "tag_detections_image",
                Image,
                self.on_image,
                queue_size=self.input_queue_size,
            )
        rospy.loginfo("loaded box overlay config: %s", config_file)
        rospy.loginfo("loaded robot overlay config: %s", robot_config_file)
        rospy.loginfo("loaded tag overlay config: %s", tag_config_file)
        if self.overlay_enabled:
            rospy.loginfo(
                "synchronizing tag detections and detection images by exact header stamp with queue size %d",
                self.sync_queue_size,
            )
        else:
            rospy.loginfo("box image overlay disabled; box pose and marker outputs remain enabled")

    def build_robot_tag_ids(self, config):
        tag_ids = set()
        robot = config.get("robot", {})
        for tag in robot.get("tags", []):
            tag_ids.add(int(tag["id"]))
        return tag_ids

    def warn_overlapping_tag_ids(self):
        box_tag_ids = set(self.pose_estimator.tag_to_box.keys())
        overlapping_ids = sorted(box_tag_ids.intersection(self.robot_tag_ids))
        if overlapping_ids:
            rospy.logwarn(
                "tag ids assigned to both boxes and robot; detections may be ambiguous: %s",
                overlapping_ids,
            )

    def load_tag_sizes(self, path):
        config = load_config(path)
        tag_sizes = {}
        for tag in config.get("standalone_tags", []):
            tag_id = int(tag["id"])
            tag_sizes[tag_id] = float(tag["size"])
        return tag_sizes

    def on_camera_info(self, msg):
        self.camera_info = msg

    def on_tag_detections(self, msg):
        box_results = self.pose_estimator.on_detections(msg)
        if not self.overlay_enabled:
            return

        synced = self.store_and_take_synced(
            self.detection_results,
            msg.header.stamp,
            (msg, box_results),
        )
        if synced:
            self.publish_synced_image(*synced)

    def on_image(self, msg):
        synced = self.store_and_take_synced(self.images, msg.header.stamp, msg)
        if synced:
            self.publish_synced_image(*synced)

    def store_and_take_synced(self, messages, header_stamp, value):
        stamp = header_stamp.to_nsec()
        with self.sync_lock:
            messages[stamp] = value
            messages.move_to_end(stamp)
            while len(messages) > self.sync_queue_size:
                messages.popitem(last=False)

            if stamp not in self.images or stamp not in self.detection_results:
                return None

            tag_detections, box_result = self.detection_results.pop(stamp)
            return (
                self.images.pop(stamp),
                tag_detections,
                box_result,
            )

    def publish_synced_image(self, image_msg, tag_detections, box_results):
        with self.render_lock:
            try:
                image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            except Exception as error:
                rospy.logwarn_throttle(2.0, "failed to convert image: %s", error)
                return

            output = image.copy()
            if self.camera_info:
                for box_result in box_results:
                    self.draw_box(
                        output,
                        box_result["camera_box"],
                        box_result["box"],
                        box_result["box_name"],
                        box_result["source_tag_id"],
                    )
                self.draw_robot_tags(output, tag_detections)
            else:
                cv2.putText(output, "no camera info", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 180, 255), 2)

            out_msg = self.bridge.cv2_to_imgmsg(output, encoding="bgr8")
            out_msg.header = image_msg.header
            self.image_pub.publish(out_msg)

    def draw_box(self, image, transform, box, box_name, source_tag_id):
        corners = [transform_point(transform, corner) for corner in self.box_corners(box)]
        pixels = [self.project(point) for point in corners]

        for edge, color in BOX_EDGES:
            start, end = edge
            if pixels[start] is None or pixels[end] is None:
                continue
            cv2.line(image, pixels[start], pixels[end], color, 2, cv2.LINE_AA)

        center = self.project((transform[0][3], transform[1][3], transform[2][3]))
        if center:
            cv2.circle(image, center, 4, (0, 0, 255), -1)
            label = "%s from tag %s" % (box_name, source_tag_id)
            cv2.putText(image, label, (center[0] + 8, center[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            self.draw_axes(image, transform, center, box)

    def draw_axes(self, image, transform, center, box):
        sx = float(box["size"]["x"])
        sy = float(box["size"]["y"])
        sz = float(box["size"]["z"])
        axis_length = max(min(sx, sy, sz) * 0.8, 0.04)

        for label, axis, color in BOX_AXES:
            endpoint = transform_point(transform, (
                axis[0] * axis_length,
                axis[1] * axis_length,
                axis[2] * axis_length,
            ))
            pixel = self.project(endpoint)
            if pixel is None:
                continue
            cv2.line(image, center, pixel, color, 3, cv2.LINE_AA)
            cv2.putText(image, label, (pixel[0] + 4, pixel[1] - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)

    def draw_robot_tags(self, image, tag_detections):
        for detection in tag_detections.detections:
            if not detection.id:
                continue
            tag_id = int(detection.id[0])
            if tag_id not in self.robot_tag_ids:
                continue

            tag_size = self.tag_sizes.get(tag_id)
            if detection.size:
                tag_size = float(detection.size[0])
            if tag_size is None:
                rospy.logwarn_throttle(2.0, "tag id %s has no size in tag config", tag_id)
                continue

            transform = pose_to_transform(detection.pose.pose.pose)
            half = tag_size / 2.0
            corners = [
                (-half, -half, 0.0),
                (half, -half, 0.0),
                (half, half, 0.0),
                (-half, half, 0.0),
            ]
            pixels = [self.project(transform_point(transform, corner)) for corner in corners]
            if any(pixel is None for pixel in pixels):
                continue

            color = (0, 255, 255)
            for index in range(4):
                cv2.line(image, pixels[index], pixels[(index + 1) % 4], color, 2, cv2.LINE_AA)

            center = self.project((transform[0][3], transform[1][3], transform[2][3]))
            if center:
                cv2.circle(image, center, 4, color, -1)
                cv2.putText(image, "robot tag %s" % tag_id, (center[0] + 8, center[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

    def project(self, point):
        x, y, z = point
        if z <= 0.0:
            return None
        k = self.camera_info.K
        fx = k[0]
        fy = k[4]
        cx = k[2]
        cy = k[5]
        if fx == 0.0 or fy == 0.0:
            rospy.logwarn_throttle(2.0, "camera_info has invalid fx/fy")
            return None
        u = int(round((fx * x / z) + cx))
        v = int(round((fy * y / z) + cy))
        return (u, v)

    def box_corners(self, box):
        sx = float(box["size"]["x"])
        sy = float(box["size"]["y"])
        sz = float(box["size"]["z"])
        hx = sx / 2.0
        hy = sy / 2.0
        hz = sz / 2.0
        return [
            (-hx, -hy, -hz),
            (hx, -hy, -hz),
            (hx, hy, -hz),
            (-hx, hy, -hz),
            (-hx, -hy, hz),
            (hx, -hy, hz),
            (hx, hy, hz),
            (-hx, hy, hz),
        ]


def main():
    rospy.init_node("box_detection_node")
    BoxDetectionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
