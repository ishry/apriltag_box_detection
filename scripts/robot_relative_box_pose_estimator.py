#!/usr/bin/env python3

import math
import os

import rospy
import rospkg
import yaml
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Point, PoseStamped, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


BOX_EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
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


def matrix_to_quaternion(m):
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m[2][1] - m[1][2]) / s
        qy = (m[0][2] - m[2][0]) / s
        qz = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        qw = (m[2][1] - m[1][2]) / s
        qx = 0.25 * s
        qy = (m[0][1] + m[1][0]) / s
        qz = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        qw = (m[0][2] - m[2][0]) / s
        qx = (m[0][1] + m[1][0]) / s
        qy = 0.25 * s
        qz = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        qw = (m[1][0] - m[0][1]) / s
        qx = (m[0][2] + m[2][0]) / s
        qy = (m[1][2] + m[2][1]) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


def quaternion_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def make_transform(position, quaternion):
    rotation = quaternion_to_matrix(quaternion)
    return [
        [rotation[0][0], rotation[0][1], rotation[0][2], position[0]],
        [rotation[1][0], rotation[1][1], rotation[1][2], position[1]],
        [rotation[2][0], rotation[2][1], rotation[2][2], position[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def multiply_transform(a, b):
    result = [[0.0 for _ in range(4)] for _ in range(4)]
    for row in range(4):
        for col in range(4):
            for index in range(4):
                result[row][col] += a[row][index] * b[index][col]
    return result


def inverse_transform(t):
    result = [
        [t[0][0], t[1][0], t[2][0], 0.0],
        [t[0][1], t[1][1], t[2][1], 0.0],
        [t[0][2], t[1][2], t[2][2], 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    translation = (t[0][3], t[1][3], t[2][3])
    result[0][3] = -(result[0][0] * translation[0] + result[0][1] * translation[1] + result[0][2] * translation[2])
    result[1][3] = -(result[1][0] * translation[0] + result[1][1] * translation[1] + result[1][2] * translation[2])
    result[2][3] = -(result[2][0] * translation[0] + result[2][1] * translation[1] + result[2][2] * translation[2])
    return result


def transform_point(t, point):
    return (
        t[0][0] * point[0] + t[0][1] * point[1] + t[0][2] * point[2] + t[0][3],
        t[1][0] * point[0] + t[1][1] * point[1] + t[1][2] * point[2] + t[1][3],
        t[2][0] * point[0] + t[2][1] * point[1] + t[2][2] * point[2] + t[2][3],
    )


def transform_to_pose(t):
    position = (t[0][3], t[1][3], t[2][3])
    rotation = [
        [t[0][0], t[0][1], t[0][2]],
        [t[1][0], t[1][1], t[1][2]],
        [t[2][0], t[2][1], t[2][2]],
    ]
    return position, matrix_to_quaternion(rotation)


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
    return make_transform(position, quaternion)


def pose_config_to_transform(config):
    position_config = config.get("position", {})
    rotation_config = config.get("rotation", {})
    position = (
        float(position_config.get("x", 0.0)),
        float(position_config.get("y", 0.0)),
        float(position_config.get("z", 0.0)),
    )
    quaternion = quaternion_from_rpy(
        float(rotation_config.get("roll", 0.0)),
        float(rotation_config.get("pitch", 0.0)),
        float(rotation_config.get("yaw", 0.0)),
    )
    return make_transform(position, quaternion)


def color_msg(r, g, b, a):
    color = ColorRGBA()
    color.r = r
    color.g = g
    color.b = b
    color.a = a
    return color


def point_msg(value):
    point = Point()
    point.x = value[0]
    point.y = value[1]
    point.z = value[2]
    return point


def marker_base(frame_id, namespace, marker_id, marker_type, stamp):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    return marker


def load_config(path):
    with open(path, "r") as config_file:
        config = yaml.safe_load(config_file)
    if not config:
        raise ValueError("empty config file: %s" % path)
    return config


def default_config_path(filename):
    package_path = rospkg.RosPack().get_path("apriltag_box_detection")
    return os.path.join(package_path, "config", filename)


class RobotRelativeBoxPoseEstimator:
    def __init__(self):
        robot_config_file = rospy.get_param("~robot_config_file", default_config_path("robot.yaml"))
        box_config_file = rospy.get_param("~box_config_file", default_config_path("box.yaml"))
        self.robot_config = load_config(robot_config_file)
        self.box_config = load_config(box_config_file)
        self.robot_frame_id = self.robot_config.get("frame_id", "robot_config")
        self.robot_tags = self.build_robot_tag_index(self.robot_config)
        self.box = self.box_config.get("boxes", [])[0]
        self.last_camera_robot = None
        self.last_robot_stamp = None
        self.last_robot_tag_id = None

        self.pose_pub = rospy.Publisher("robot_relative_box_pose", PoseStamped, queue_size=1)
        self.marker_pub = rospy.Publisher("robot_relative_box_markers", MarkerArray, queue_size=1)
        self.tag_subscriber = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.on_detections, queue_size=1)
        self.box_subscriber = rospy.Subscriber("box_pose", PoseStamped, self.on_box_pose, queue_size=1)

        rospy.loginfo("loaded robot config: %s", robot_config_file)
        rospy.loginfo("loaded box config: %s", box_config_file)
        rospy.loginfo("indexed %d robot tag(s) for robot-relative box pose", len(self.robot_tags))

    def build_robot_tag_index(self, config):
        tag_to_robot = {}
        robot = config.get("robot", {})
        for tag in robot.get("tags", []):
            tag_id = int(tag["id"])
            if tag_id in tag_to_robot:
                raise ValueError("duplicate tag id in robot config: %s" % tag_id)
            tag_to_robot[tag_id] = pose_config_to_transform(tag.get("pose", {}))
        return tag_to_robot

    def on_detections(self, msg):
        for detection in msg.detections:
            if not detection.id:
                continue
            tag_id = int(detection.id[0])
            if tag_id not in self.robot_tags:
                continue

            camera_tag = pose_to_transform(detection.pose.pose.pose)
            robot_tag = self.robot_tags[tag_id]
            self.last_camera_robot = multiply_transform(camera_tag, inverse_transform(robot_tag))
            self.last_robot_stamp = msg.header.stamp
            self.last_robot_tag_id = tag_id
            return

    def on_box_pose(self, msg):
        if self.last_camera_robot is None:
            rospy.logwarn_throttle(2.0, "no robot tag pose has been observed yet; robot-relative box pose is not available")
            return

        camera_box = pose_to_transform(msg.pose)
        robot_box = multiply_transform(inverse_transform(self.last_camera_robot), camera_box)
        self.publish_robot_relative_box_pose(msg.header.stamp, robot_box)
        self.publish_robot_relative_markers(msg.header.stamp, robot_box)

    def publish_robot_relative_box_pose(self, stamp, robot_box):
        position, quaternion = transform_to_pose(robot_box)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.robot_frame_id
        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = position[2]
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        self.pose_pub.publish(pose_msg)

    def publish_robot_relative_markers(self, stamp, robot_box):
        marker_array = MarkerArray()
        marker_id = 0
        size = self.box["size"]
        sx = float(size["x"])
        sy = float(size["y"])
        sz = float(size["z"])
        position, quaternion = transform_to_pose(robot_box)

        cube = marker_base(self.robot_frame_id, "robot_relative_box", marker_id, Marker.CUBE, stamp)
        marker_id += 1
        cube.pose.position.x = position[0]
        cube.pose.position.y = position[1]
        cube.pose.position.z = position[2]
        cube.pose.orientation.x = quaternion[0]
        cube.pose.orientation.y = quaternion[1]
        cube.pose.orientation.z = quaternion[2]
        cube.pose.orientation.w = quaternion[3]
        cube.scale = Vector3(sx, sy, sz)
        cube.color = color_msg(0.1, 0.45, 1.0, 0.18)
        marker_array.markers.append(cube)

        line = marker_base(self.robot_frame_id, "robot_relative_box_edges", marker_id, Marker.LINE_LIST, stamp)
        marker_id += 1
        line.scale.x = 0.006
        line.color = color_msg(0.0, 1.0, 0.35, 1.0)
        corners = self.box_corners(sx, sy, sz)
        transformed = [transform_point(robot_box, corner) for corner in corners]
        for start, end in BOX_EDGES:
            line.points.append(point_msg(transformed[start]))
            line.points.append(point_msg(transformed[end]))
        marker_array.markers.append(line)

        text = marker_base(self.robot_frame_id, "robot_relative_box_label", marker_id, Marker.TEXT_VIEW_FACING, stamp)
        text.pose.position.x = position[0]
        text.pose.position.y = position[1]
        text.pose.position.z = position[2] + sz / 2.0 + 0.04
        text.scale.z = 0.035
        text.color = color_msg(1.0, 1.0, 1.0, 1.0)
        text.text = "%s in robot frame" % self.box.get("name", "box")
        if self.last_robot_tag_id is not None:
            text.text += " from robot tag %s" % self.last_robot_tag_id
        marker_array.markers.append(text)

        self.marker_pub.publish(marker_array)

    def box_corners(self, sx, sy, sz):
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
    rospy.init_node("robot_relative_box_pose_estimator")
    RobotRelativeBoxPoseEstimator()
    rospy.spin()


if __name__ == "__main__":
    main()
