#!/usr/bin/env python3

import math
import os

import rospy
import rospkg
import yaml
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


FACE_AXES = {
    0: {"normal": (1.0, 0.0, 0.0), "u": (0.0, 1.0, 0.0), "v": (0.0, 0.0, 1.0), "label": "+X"},
    1: {"normal": (-1.0, 0.0, 0.0), "u": (0.0, 1.0, 0.0), "v": (0.0, 0.0, -1.0), "label": "-X"},
    2: {"normal": (0.0, 1.0, 0.0), "u": (-1.0, 0.0, 0.0), "v": (0.0, 0.0, 1.0), "label": "+Y"},
    3: {"normal": (0.0, -1.0, 0.0), "u": (1.0, 0.0, 0.0), "v": (0.0, 0.0, 1.0), "label": "-Y"},
    4: {"normal": (0.0, 0.0, 1.0), "u": (1.0, 0.0, 0.0), "v": (0.0, 1.0, 0.0), "label": "+Z"},
    5: {"normal": (0.0, 0.0, -1.0), "u": (1.0, 0.0, 0.0), "v": (0.0, -1.0, 0.0), "label": "-Z"},
}


def vec_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vec_scale(a, scale):
    return (a[0] * scale, a[1] * scale, a[2] * scale)


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


def rotated_face_axes(face, yaw):
    axes = FACE_AXES[face]
    u = axes["u"]
    v = axes["v"]
    n = axes["normal"]
    c = math.cos(yaw)
    s = math.sin(yaw)
    tag_x = vec_add(vec_scale(u, c), vec_scale(v, s))
    tag_y = vec_add(vec_scale(u, -s), vec_scale(v, c))
    return tag_x, tag_y, n


def orientation_from_axes(x_axis, y_axis, z_axis):
    matrix = [
        [x_axis[0], y_axis[0], z_axis[0]],
        [x_axis[1], y_axis[1], z_axis[1]],
        [x_axis[2], y_axis[2], z_axis[2]],
    ]
    return matrix_to_quaternion(matrix)


def point_msg(value):
    point = Point()
    point.x = value[0]
    point.y = value[1]
    point.z = value[2]
    return point


def color_msg(r, g, b, a):
    color = ColorRGBA()
    color.r = r
    color.g = g
    color.b = b
    color.a = a
    return color


def set_pose(marker, position, quaternion):
    marker.pose.position = point_msg(position)
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]


def marker_base(frame_id, namespace, marker_id, marker_type):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    return marker


def box_half_size(box):
    size = box["size"]
    return (float(size["x"]) / 2.0, float(size["y"]) / 2.0, float(size["z"]) / 2.0)


def face_center(box, face):
    half = box_half_size(box)
    normal = FACE_AXES[face]["normal"]
    return (normal[0] * half[0], normal[1] * half[1], normal[2] * half[2])


def tag_pose(box, tag):
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
    return position, quaternion, u_axis, v_axis, normal_axis


def add_arrow(markers, frame_id, namespace, marker_id, start, axis, length, color):
    marker = marker_base(frame_id, namespace, marker_id, Marker.ARROW)
    marker.points = [
        point_msg(start),
        point_msg(vec_add(start, vec_scale(axis, length))),
    ]
    marker.scale = Vector3(0.006, 0.014, 0.0)
    marker.color = color
    markers.markers.append(marker)


def tag_mesh_path(package_path, tag_id):
    mesh_path = os.path.join(package_path, "assets", "meshes", "tag_%s.dae" % tag_id)
    if os.path.exists(mesh_path):
        return "package://apriltag_box_detection/assets/meshes/tag_%s.dae" % tag_id
    return None


def add_tag_marker(markers, frame_id, marker_id, position, quaternion, tag_size, mesh_resource):
    if mesh_resource:
        marker = marker_base(frame_id, "tag_mesh", marker_id, Marker.MESH_RESOURCE)
        marker.mesh_resource = mesh_resource
        marker.mesh_use_embedded_materials = True
        marker.scale = Vector3(tag_size, tag_size, 1.0)
        marker.color = color_msg(1.0, 1.0, 1.0, 1.0)
    else:
        marker = marker_base(frame_id, "tag", marker_id, Marker.CUBE)
        marker.scale = Vector3(tag_size, tag_size, 0.004)
        marker.color = color_msg(0.05, 0.05, 0.05, 1.0)

    set_pose(marker, position, quaternion)
    markers.markers.append(marker)


def build_markers(config, package_path):
    frame_id = config.get("frame_id", "box_config")
    markers = MarkerArray()
    marker_id = 0

    for box in config.get("boxes", []):
        name = box.get("name", "box")
        size = box["size"]

        box_marker = marker_base(frame_id, "box", marker_id, Marker.CUBE)
        marker_id += 1
        set_pose(box_marker, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        box_marker.scale = Vector3(float(size["x"]), float(size["y"]), float(size["z"]))
        box_marker.color = color_msg(0.1, 0.45, 1.0, 0.22)
        markers.markers.append(box_marker)

        name_marker = marker_base(frame_id, "box_name", marker_id, Marker.TEXT_VIEW_FACING)
        marker_id += 1
        set_pose(name_marker, (0.0, 0.0, float(size["z"]) / 2.0 + 0.05), (0.0, 0.0, 0.0, 1.0))
        name_marker.scale.z = 0.035
        name_marker.color = color_msg(1.0, 1.0, 1.0, 1.0)
        name_marker.text = name
        markers.markers.append(name_marker)

        for face in sorted(FACE_AXES.keys()):
            center = face_center(box, face)
            label_pos = vec_add(center, vec_scale(FACE_AXES[face]["normal"], 0.035))
            label = marker_base(frame_id, "face_label", marker_id, Marker.TEXT_VIEW_FACING)
            marker_id += 1
            set_pose(label, label_pos, (0.0, 0.0, 0.0, 1.0))
            label.scale.z = 0.025
            label.color = color_msg(1.0, 0.9, 0.1, 1.0)
            label.text = "%d %s" % (face, FACE_AXES[face]["label"])
            markers.markers.append(label)

        for tag in box.get("tags", []):
            position, quaternion, tag_x, tag_y, tag_z = tag_pose(box, tag)
            tag_size = float(tag["size"])

            mesh_resource = tag_mesh_path(package_path, tag["id"])
            add_tag_marker(markers, frame_id, marker_id, position, quaternion, tag_size, mesh_resource)
            marker_id += 1

            text_pos = vec_add(position, vec_scale(tag_z, 0.025))
            text = marker_base(frame_id, "tag_label", marker_id, Marker.TEXT_VIEW_FACING)
            marker_id += 1
            set_pose(text, text_pos, (0.0, 0.0, 0.0, 1.0))
            text.scale.z = 0.025
            text.color = color_msg(1.0, 1.0, 1.0, 1.0)
            text.text = "tag %s" % tag["id"]
            markers.markers.append(text)

            axis_length = max(tag_size * 0.8, 0.04)
            add_arrow(markers, frame_id, "tag_axis", marker_id, position, tag_x, axis_length, color_msg(1.0, 0.1, 0.1, 1.0))
            marker_id += 1
            add_arrow(markers, frame_id, "tag_axis", marker_id, position, tag_y, axis_length, color_msg(0.1, 1.0, 0.1, 1.0))
            marker_id += 1
            add_arrow(markers, frame_id, "tag_axis", marker_id, position, tag_z, axis_length, color_msg(0.1, 0.35, 1.0, 1.0))
            marker_id += 1

    return markers


def load_config(path):
    with open(path, "r") as config_file:
        config = yaml.safe_load(config_file)
    if not config:
        raise ValueError("empty config file: %s" % path)
    return config


def default_config_path():
    package_path = rospkg.RosPack().get_path("apriltag_box_detection")
    return package_path + "/config/box.yaml"


def main():
    rospy.init_node("box_config_viewer")
    config_file = rospy.get_param("~config_file", default_config_path())
    publish_rate = float(rospy.get_param("~publish_rate", 1.0))
    publisher = rospy.Publisher("box_config_markers", MarkerArray, queue_size=1, latch=True)
    package_path = rospkg.RosPack().get_path("apriltag_box_detection")

    try:
        config = load_config(config_file)
        markers = build_markers(config, package_path)
    except Exception as error:
        rospy.logerr("failed to load box config '%s': %s", config_file, error)
        raise

    rospy.loginfo("loaded box config: %s", config_file)
    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        for marker in markers.markers:
            marker.header.stamp = now
        publisher.publish(markers)
        rate.sleep()


if __name__ == "__main__":
    main()
