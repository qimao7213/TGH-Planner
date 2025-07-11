#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 这个脚本的主要功能是将传感器的里程计数据转换为tf变换，并发布到ROS中。

import rospy
import tf
import tf.transformations as tft
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
import time

last_time = time.time()

# 全局变量，存储 base_link 到 lidar_link 的静态变换
base_to_lidar_trans = None
base_to_lidar_rot = None
lidar_frame = "jackal/velodyne/VLP_16_base_link"
base_frame = "base_link"
world_frame = "world"
sensor_odom_topic = "/jackal/velodyne/gazebo_gt/odometry"

# lidar_frame = "body"
# base_frame = "base_link"
# world_frame = "world"
# sensor_odom_topic = "/Odometry"

br = tf2_ros.TransformBroadcaster()

def query_base_to_lidar_transform(tf_listener):
    global base_to_lidar_trans, base_to_lidar_rot
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = tf_listener.lookupTransform(base_frame, lidar_frame, rospy.Time(0))
            base_to_lidar_trans = np.array(trans)
            base_to_lidar_rot = np.array(rot)
            rospy.loginfo("Got static transform from %s to %s", base_frame, lidar_frame)
            return
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Waiting for static tf from %s to %s...", base_frame, lidar_frame)
            rate.sleep()

def odom_callback(msg):
    global base_to_lidar_trans, base_to_lidar_rot
    if base_to_lidar_trans is None or base_to_lidar_rot is None:
        rospy.logwarn("Static tf not ready, skip this odom msg.")
        return
    global last_time

    if time.time() - last_time < 0.08:  # 10 Hz 限制
        return

    last_time = time.time()

    # lidar_link在world下的位姿
    lidar_pos = np.array([
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z
    ])
    lidar_rot = np.array([
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    ])

    # 构造T_world_lidar
    T_world_lidar = tft.concatenate_matrices(
        tft.translation_matrix(lidar_pos),
        tft.quaternion_matrix(lidar_rot)
    )
    # 构造T_base_lidar
    T_base_lidar = tft.concatenate_matrices(
        tft.translation_matrix(base_to_lidar_trans),
        tft.quaternion_matrix(base_to_lidar_rot)
    )
    # 求逆得到T_lidar_base
    T_lidar_base = tft.inverse_matrix(T_base_lidar)
    # 计算T_world_base
    T_world_base = np.dot(T_world_lidar, T_lidar_base)
    # 提取base_link在world下的平移和旋转
    trans = tft.translation_from_matrix(T_world_base)
    quat = tft.quaternion_from_matrix(T_world_base)

    # 发布world->base_link的tf
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = world_frame
    transform.child_frame_id = base_frame
    transform.transform.translation.x = trans[0]
    transform.transform.translation.y = trans[1]
    transform.transform.translation.z = trans[2]
    transform.transform.rotation.x = quat[0]
    transform.transform.rotation.y = quat[1]
    transform.transform.rotation.z = quat[2]
    transform.transform.rotation.w = quat[3]
    br.sendTransform(transform)

def main():
    rospy.init_node('odom_to_tf')
    global lidar_frame, base_frame, world_frame, sensor_odom_topic
    lidar_frame = rospy.get_param('~lidar_frame', 'jackal/velodyne/VLP_16_base_link')
    base_frame = rospy.get_param('~base_frame', 'base_link')
    sensor_odom_topic = rospy.get_param('~sensor_odom_topic', '/jackal/velodyne/gazebo_gt/odometry')
    tf_listener = tf.TransformListener()
    query_base_to_lidar_transform(tf_listener)
    rospy.Subscriber(sensor_odom_topic, Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    main()