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
from apriltag_box_detection.msg import BoxPose, BoxPoseArray


FACE_AXES = {
    0: {"normal": (1.0, 0.0, 0.0), "u": (0.0, 1.0, 0.0), "v": (0.0, 0.0, 1.0)},
    1: {"normal": (-1.0, 0.0, 0.0), "u": (0.0, 1.0, 0.0), "v": (0.0, 0.0, -1.0)},
    2: {"normal": (0.0, 1.0, 0.0), "u": (-1.0, 0.0, 0.0), "v": (0.0, 0.0, 1.0)},
    3: {"normal": (0.0, -1.0, 0.0), "u": (1.0, 0.0, 0.0), "v": (0.0, 0.0, 1.0)},
    4: {"normal": (0.0, 0.0, 1.0), "u": (1.0, 0.0, 0.0), "v": (0.0, 1.0, 0.0)},
    5: {"normal": (0.0, 0.0, -1.0), "u": (1.0, 0.0, 0.0), "v": (0.0, -1.0, 0.0)},
}


BOX_EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
]


def vec_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vec_scale(a, scale):
    return (a[0] * scale, a[1] * scale, a[2] * scale)


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


def rotated_face_axes(face, yaw):
    axes = FACE_AXES[face]
    u = axes["u"]
    v = axes["v"]
    normal = axes["normal"]
    c = math.cos(yaw)
    s = math.sin(yaw)
    tag_x = vec_add(vec_scale(u, c), vec_scale(v, s))
    tag_y = vec_add(vec_scale(u, -s), vec_scale(v, c))
    return tag_x, tag_y, normal


def orientation_from_axes(x_axis, y_axis, z_axis):
    matrix = [
        [x_axis[0], y_axis[0], z_axis[0]],
        [x_axis[1], y_axis[1], z_axis[1]],
        [x_axis[2], y_axis[2], z_axis[2]],
    ]
    return matrix_to_quaternion(matrix)


def box_half_size(box):
    size = box["size"]
    return (float(size["x"]) / 2.0, float(size["y"]) / 2.0, float(size["z"]) / 2.0)


def face_center(box, face):
    half = box_half_size(box)
    normal = FACE_AXES[face]["normal"]
    return (normal[0] * half[0], normal[1] * half[1], normal[2] * half[2])


def tag_transform_in_box(box, tag):
    face = int(tag["face"])
    if face not in FACE_AXES:
        raise ValueError("tag id %s has invalid face %s" % (tag.get("id"), face))

    offset = tag.get("offset", {})
    yaw = float(tag.get("rotation", {}).get("yaw", 0.0))
    u_axis, v_axis, normal_axis = rotated_face_axes(face, yaw)
    position = face_center(box, face)
    position = vec_add(position, vec_scale(FACE_AXES[face]["u"], float(offset.get("u", 0.0))))
    position = vec_add(position, vec_scale(FACE_AXES[face]["v"], float(offset.get("v", 0.0))))
    position = vec_add(position, vec_scale(FACE_AXES[face]["normal"], float(offset.get("normal", 0.0))))
    quaternion = orientation_from_axes(u_axis, v_axis, normal_axis)
    return make_transform(position, quaternion)


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


def default_config_path():
    package_path = rospkg.RosPack().get_path("apriltag_box_detection")
    return os.path.join(package_path, "config", "box.yaml")


class BoxPoseEstimator:
    def __init__(self, subscribe=True):
        config_file = rospy.get_param("~config_file", default_config_path())
        self.config = load_config(config_file)
        self.tag_to_box = self.build_tag_index(self.config)
        self.pose_array_pub = rospy.Publisher("box_poses", BoxPoseArray, queue_size=1)
        self.pose_pub = rospy.Publisher("box_pose", PoseStamped, queue_size=1)
        self.marker_pub = rospy.Publisher("box_detection_markers", MarkerArray, queue_size=1)
        self.previous_marker_box_ids = set()
        self.subscriber = None
        if subscribe:
            queue_size = max(1, int(rospy.get_param("~queue_size", 10)))
            self.subscriber = rospy.Subscriber(
                "tag_detections",
                AprilTagDetectionArray,
                self.on_detections,
                queue_size=queue_size,
            )
        rospy.loginfo("loaded box config: %s", config_file)
        rospy.loginfo("indexed %d tag(s) for box pose estimation", len(self.tag_to_box))

    def build_tag_index(self, config):
        tag_to_box = {}
        box_ids = set()
        for index, box in enumerate(config.get("boxes", [])):
            box_id = str(box.get("id", box.get("name", "box_%d" % index)))
            box_name = str(box.get("name", box_id))
            if box_id in box_ids:
                raise ValueError("duplicate box id in box config: %s" % box_id)
            box_ids.add(box_id)

            for tag in box.get("tags", []):
                tag_id = int(tag["id"])
                if tag_id in tag_to_box:
                    raise ValueError("duplicate tag id in box config: %s" % tag_id)
                tag_to_box[tag_id] = {
                    "box": box,
                    "box_id": box_id,
                    "box_name": box_name,
                    "tag": tag,
                    "box_tag_transform": tag_transform_in_box(box, tag),
                }
        return tag_to_box

    def on_detections(self, msg):
        results = self.estimate_boxes(msg)
        self.publish_box_poses(msg.header, results)
        self.publish_markers(msg.header, results)
        return results

    def estimate_box(self, msg):
        results = self.estimate_boxes(msg)
        if not results:
            return None
        return results[0]

    def estimate_boxes(self, msg):
        results_by_box_id = {}
        for detection in msg.detections:
            if not detection.id:
                continue
            tag_id = int(detection.id[0])
            if tag_id not in self.tag_to_box:
                continue

            box_data = self.tag_to_box[tag_id]
            box_id = box_data["box_id"]
            if box_id in results_by_box_id:
                continue
            camera_tag = pose_to_transform(detection.pose.pose.pose)
            box_tag = box_data["box_tag_transform"]
            camera_box = multiply_transform(camera_tag, inverse_transform(box_tag))
            results_by_box_id[box_id] = {
                "camera_box": camera_box,
                "box": box_data["box"],
                "box_id": box_id,
                "box_name": box_data["box_name"],
                "source_tag_id": tag_id,
            }
        return list(results_by_box_id.values())

    def publish_box_poses(self, header, results):
        pose_array = BoxPoseArray()
        pose_array.header = header

        for result in results:
            box_pose = BoxPose()
            box_pose.box_id = result["box_id"]
            box_pose.box_name = result["box_name"]
            box_pose.source_tag_id = result["source_tag_id"]
            self.set_pose(box_pose.pose, result["camera_box"])
            pose_array.boxes.append(box_pose)

        self.pose_array_pub.publish(pose_array)

        # Keep the original single-pose topic for existing consumers. New
        # consumers should use box_poses so that no detections are lost.
        if results:
            self.publish_box_pose(header, results[0]["camera_box"])

    def set_pose(self, pose, transform):
        position, quaternion = transform_to_pose(transform)
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

    def publish_box_pose(self, header, camera_box):
        position, quaternion = transform_to_pose(camera_box)
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = position[2]
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        self.pose_pub.publish(pose_msg)

    def publish_markers(self, header, results):
        marker_array = MarkerArray()
        current_box_ids = set()

        for result in results:
            box_id = result["box_id"]
            current_box_ids.add(box_id)
            box = result["box"]
            camera_box = result["camera_box"]
            namespace = "detected_box_%s" % box_id
            size = box["size"]
            sx = float(size["x"])
            sy = float(size["y"])
            sz = float(size["z"])

            cube = marker_base(header.frame_id, namespace, 0, Marker.CUBE, header.stamp)
            self.set_pose(cube.pose, camera_box)
            cube.scale = Vector3(sx, sy, sz)
            cube.color = color_msg(0.1, 0.45, 1.0, 0.18)
            marker_array.markers.append(cube)

            line = marker_base(header.frame_id, namespace + "_edges", 0, Marker.LINE_LIST, header.stamp)
            line.scale.x = 0.006
            line.color = color_msg(0.0, 1.0, 0.35, 1.0)
            corners = self.box_corners(sx, sy, sz)
            transformed = [transform_point(camera_box, corner) for corner in corners]
            for start, end in BOX_EDGES:
                line.points.append(point_msg(transformed[start]))
                line.points.append(point_msg(transformed[end]))
            marker_array.markers.append(line)

            text = marker_base(header.frame_id, namespace + "_label", 0, Marker.TEXT_VIEW_FACING, header.stamp)
            position, quaternion = transform_to_pose(camera_box)
            text.pose.position.x = position[0]
            text.pose.position.y = position[1]
            text.pose.position.z = position[2] + sz / 2.0 + 0.04
            text.scale.z = 0.035
            text.color = color_msg(1.0, 1.0, 1.0, 1.0)
            text.text = "%s from tag %s" % (result["box_name"], result["source_tag_id"])
            marker_array.markers.append(text)

        for box_id in sorted(self.previous_marker_box_ids - current_box_ids):
            namespace = "detected_box_%s" % box_id
            for marker_namespace in (namespace, namespace + "_edges", namespace + "_label"):
                marker = marker_base(header.frame_id, marker_namespace, 0, Marker.CUBE, header.stamp)
                marker.action = Marker.DELETE
                marker_array.markers.append(marker)

        self.previous_marker_box_ids = current_box_ids

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
    rospy.init_node("box_pose_estimator")
    BoxPoseEstimator()
    rospy.spin()


if __name__ == "__main__":
    main()
