#!/usr/bin/env python3

import math
import os

import rospy
import rospkg
import yaml
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


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


def vec_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vec_scale(a, scale):
    return (a[0] * scale, a[1] * scale, a[2] * scale)


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


def quaternion_to_axes(quaternion):
    x, y, z, w = quaternion
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy + wz), 2.0 * (xz - wy)),
        (2.0 * (xy - wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz + wx)),
        (2.0 * (xz + wy), 2.0 * (yz - wx), 1.0 - 2.0 * (xx + yy)),
    )


def marker_base(frame_id, namespace, marker_id, marker_type):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    return marker


def set_pose(marker, position, quaternion):
    marker.pose.position = point_msg(position)
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]


def pose_from_config(config):
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
    return position, quaternion


def scale_from_config(config, default):
    return Vector3(
        float(config.get("x", default[0])),
        float(config.get("y", default[1])),
        float(config.get("z", default[2])),
    )


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
        marker = marker_base(frame_id, "robot_tag_mesh", marker_id, Marker.MESH_RESOURCE)
        marker.mesh_resource = mesh_resource
        marker.mesh_use_embedded_materials = True
        marker.scale = Vector3(tag_size, tag_size, 1.0)
        marker.color = color_msg(1.0, 1.0, 1.0, 1.0)
    else:
        marker = marker_base(frame_id, "robot_tag", marker_id, Marker.CUBE)
        marker.scale = Vector3(tag_size, tag_size, 0.004)
        marker.color = color_msg(0.05, 0.05, 0.05, 1.0)

    set_pose(marker, position, quaternion)
    markers.markers.append(marker)


def add_axis_markers(markers, frame_id, namespace, marker_id, position, quaternion, length):
    x_axis, y_axis, z_axis = quaternion_to_axes(quaternion)
    add_arrow(markers, frame_id, namespace, marker_id, position, x_axis, length, color_msg(1.0, 0.1, 0.1, 1.0))
    marker_id += 1
    add_arrow(markers, frame_id, namespace, marker_id, position, y_axis, length, color_msg(0.1, 1.0, 0.1, 1.0))
    marker_id += 1
    add_arrow(markers, frame_id, namespace, marker_id, position, z_axis, length, color_msg(0.1, 0.35, 1.0, 1.0))
    marker_id += 1
    return marker_id


def add_text_marker(markers, frame_id, namespace, marker_id, position, text, size):
    marker = marker_base(frame_id, namespace, marker_id, Marker.TEXT_VIEW_FACING)
    set_pose(marker, position, (0.0, 0.0, 0.0, 1.0))
    marker.scale.z = size
    marker.color = color_msg(1.0, 1.0, 1.0, 1.0)
    marker.text = text
    markers.markers.append(marker)


def tag_size_from_config(tag_sizes, tag_id):
    if tag_id not in tag_sizes:
        raise ValueError("tag id %s has no size in tag config" % tag_id)
    return tag_sizes[tag_id]


def build_markers(config, package_path, tag_sizes):
    frame_id = config.get("frame_id", "robot_config")
    robot = config.get("robot", {})
    markers = MarkerArray()
    marker_id = 0

    mesh_resource = robot.get("mesh_resource", "")
    if mesh_resource:
        mesh = marker_base(frame_id, "robot_mesh", marker_id, Marker.MESH_RESOURCE)
        marker_id += 1
        mesh.mesh_resource = mesh_resource
        mesh.mesh_use_embedded_materials = True
        mesh.scale = scale_from_config(robot.get("mesh_scale", {}), (1.0, 1.0, 1.0))
        mesh.color = color_msg(0.8, 0.8, 0.8, 1.0)
        position, quaternion = pose_from_config(robot.get("mesh_pose", {}))
        set_pose(mesh, position, quaternion)
        markers.markers.append(mesh)

    add_text_marker(markers, frame_id, "robot_name", marker_id, (0.0, 0.0, 0.08), robot.get("name", "robot"), 0.04)
    marker_id += 1
    marker_id = add_axis_markers(markers, frame_id, "robot_origin_axis", marker_id, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 0.12)

    for tag in robot.get("tags", []):
        tag_id = int(tag["id"])
        tag_size = tag_size_from_config(tag_sizes, tag_id)
        position, quaternion = pose_from_config(tag.get("pose", {}))
        mesh_resource = tag_mesh_path(package_path, tag_id)
        add_tag_marker(markers, frame_id, marker_id, position, quaternion, tag_size, mesh_resource)
        marker_id += 1

        text_pos = vec_add(position, (0.0, 0.0, max(tag_size * 0.6, 0.035)))
        add_text_marker(markers, frame_id, "robot_tag_label", marker_id, text_pos, "tag %s" % tag_id, 0.025)
        marker_id += 1

        axis_length = max(tag_size * 0.8, 0.04)
        marker_id = add_axis_markers(markers, frame_id, "robot_tag_axis", marker_id, position, quaternion, axis_length)

    return markers


def load_config(path):
    with open(path, "r") as config_file:
        config = yaml.safe_load(config_file)
    if not config:
        raise ValueError("empty config file: %s" % path)
    return config


def load_tag_sizes(path):
    config = load_config(path)
    tag_sizes = {}
    for tag in config.get("standalone_tags", []):
        tag_id = int(tag["id"])
        tag_sizes[tag_id] = float(tag["size"])
    return tag_sizes


def default_config_path():
    package_path = rospkg.RosPack().get_path("apriltag_box_detection")
    return os.path.join(package_path, "config", "robot.yaml")


def default_tag_config_path():
    package_path = rospkg.RosPack().get_path("apriltag_box_detection")
    return os.path.join(package_path, "config", "tags.yaml")


def main():
    rospy.init_node("robot_config_viewer")
    config_file = rospy.get_param("~config_file", default_config_path())
    tag_config_file = rospy.get_param("~tag_config_file", default_tag_config_path())
    publish_rate = float(rospy.get_param("~publish_rate", 1.0))
    publisher = rospy.Publisher("robot_config_markers", MarkerArray, queue_size=1, latch=True)
    package_path = rospkg.RosPack().get_path("apriltag_box_detection")

    try:
        config = load_config(config_file)
        tag_sizes = load_tag_sizes(tag_config_file)
        markers = build_markers(config, package_path, tag_sizes)
    except Exception as error:
        rospy.logerr("failed to load robot config '%s': %s", config_file, error)
        raise

    rospy.loginfo("loaded robot config: %s", config_file)
    rospy.loginfo("loaded tag config: %s", tag_config_file)
    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        for marker in markers.markers:
            marker.header.stamp = now
        publisher.publish(markers)
        rate.sleep()


if __name__ == "__main__":
    main()
