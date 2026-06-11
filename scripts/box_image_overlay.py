#!/usr/bin/env python3

import math
import os

import cv2
import rospy
import rospkg
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
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


class BoxImageOverlay:
    def __init__(self):
        config_file = rospy.get_param("~config_file", default_config_path())
        self.config = load_config(config_file)
        self.box = self.config.get("boxes", [])[0]
        self.box_name = self.box.get("name", "box")
        self.box_size = self.box["size"]
        self.max_pose_age = float(rospy.get_param("~max_pose_age", 0.5))
        self.bridge = CvBridge()
        self.camera_info = None
        self.box_pose = None

        self.image_pub = rospy.Publisher("box_detection_image", Image, queue_size=1)
        self.info_sub = rospy.Subscriber("camera_info", CameraInfo, self.on_camera_info, queue_size=1)
        self.pose_sub = rospy.Subscriber("box_pose", PoseStamped, self.on_box_pose, queue_size=1)
        self.image_sub = rospy.Subscriber("image", Image, self.on_image, queue_size=1)
        rospy.loginfo("loaded box overlay config: %s", config_file)

    def on_camera_info(self, msg):
        self.camera_info = msg

    def on_box_pose(self, msg):
        self.box_pose = msg

    def on_image(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as error:
            rospy.logwarn_throttle(2.0, "failed to convert image: %s", error)
            return

        output = image.copy()
        if self.camera_info and self.box_pose and self.pose_is_fresh(msg.header.stamp, self.box_pose.header.stamp):
            self.draw_box(output, msg.header.stamp)
        else:
            cv2.putText(output, "no fresh box pose", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 180, 255), 2)

        out_msg = self.bridge.cv2_to_imgmsg(output, encoding="bgr8")
        out_msg.header = msg.header
        self.image_pub.publish(out_msg)

    def pose_is_fresh(self, image_stamp, pose_stamp):
        if pose_stamp == rospy.Time(0) or image_stamp == rospy.Time(0):
            return True
        return abs((image_stamp - pose_stamp).to_sec()) <= self.max_pose_age

    def draw_box(self, image, stamp):
        transform = pose_to_transform(self.box_pose.pose)
        corners = [transform_point(transform, corner) for corner in self.box_corners()]
        pixels = [self.project(point) for point in corners]

        for edge, color in BOX_EDGES:
            start, end = edge
            if pixels[start] is None or pixels[end] is None:
                continue
            cv2.line(image, pixels[start], pixels[end], color, 2, cv2.LINE_AA)

        center = self.project((transform[0][3], transform[1][3], transform[2][3]))
        if center:
            cv2.circle(image, center, 4, (0, 0, 255), -1)
            cv2.putText(image, self.box_name, (center[0] + 8, center[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            self.draw_axes(image, transform, center)

    def draw_axes(self, image, transform, center):
        sx = float(self.box_size["x"])
        sy = float(self.box_size["y"])
        sz = float(self.box_size["z"])
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

    def box_corners(self):
        sx = float(self.box_size["x"])
        sy = float(self.box_size["y"])
        sz = float(self.box_size["z"])
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
    rospy.init_node("box_image_overlay")
    BoxImageOverlay()
    rospy.spin()


if __name__ == "__main__":
    main()
