#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import math
import sys
import select  # 用于监听键盘输入

offset_x = 50
offset_y = 50


def wait_for_keypress():
    """等待用户按下任意键"""
    print("Press any key to publish the next set of points...")
    while True:
        # 使用 select 检测标准输入是否有数据
        if select.select([sys.stdin], [], [], 0.1)[0]:
            sys.stdin.read(1)  # 读取一个字符
            break


def generate_random_pose(x_range, y_range):
    """生成随机位姿"""
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = "world"
    pose.header.stamp = rospy.Time.now()
    pose.pose.pose.position.x = random.uniform(*x_range)
    pose.pose.pose.position.y = random.uniform(*y_range)
    pose.pose.pose.position.z = -0.8  # 固定 z 值
    pose.pose.pose.orientation.w = 1.0  # 固定朝向
    return pose

def generate_random_goal(x_range, y_range):
    """生成随机目标点"""
    goal = PoseStamped()
    goal.header.frame_id = "world"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = random.uniform(*x_range)
    goal.pose.position.y = random.uniform(*y_range)
    goal.pose.position.z = -0.8  # 固定 z 值
    goal.pose.orientation.w = 1.0  # 固定朝向
    return goal

def publish_from_file(file_path, initial_pose_pub, goal_pose_pub, rate):
    """从文件读取点并发布"""
    try:
        with open(file_path, 'r') as file:
            line_number = 0
            start_line = 0
            for line in file:
                
                if line_number < start_line:
                    line_number += 1
                    continue
                # wait_for_keypress()
                data = line.strip().split()
                if len(data) < 6:
                    rospy.logwarn("Invalid line format: %s", line)
                    continue
                # data[1] = -43.5132
                # data[2] = -11.5544
                # data[3] = 18.8547
                # data[4] = -30.482
                data[2] = -float(data[2]) #这个是因为地图是从img里面读取的，所以坐标是反的。以后要写好注释，不能这么搞了啊
                data[4] =  -float(data[4])
                

                # 从文件中读取起点和终点
                start_x, start_y = float(data[1]) + offset_x, float(data[2]) + offset_y
                goal_x, goal_y = float(data[3]) + offset_x, float(data[4]) + offset_y

                # 构造起点和终点消息
                initial_pose = PoseWithCovarianceStamped()
                initial_pose.header.frame_id = "world"
                initial_pose.header.stamp = rospy.Time.now()
                initial_pose.pose.pose.position.x = start_x
                initial_pose.pose.pose.position.y = start_y
                initial_pose.pose.pose.position.z = -0.8
                initial_pose.pose.pose.orientation.w = 1.0

                goal_pose = PoseStamped()
                goal_pose.header.frame_id = "world"
                goal_pose.header.stamp = rospy.Time.now()
                goal_pose.pose.position.x = goal_x
                goal_pose.pose.position.y = goal_y
                goal_pose.pose.position.z = -0.8
                goal_pose.pose.orientation.w = 1.0

                # 发布消息
                initial_pose_pub.publish(initial_pose)
                goal_pose_pub.publish(goal_pose)

                rospy.loginfo("Published initial pose from file: x=%.2f, y=%.2f", start_x, start_y)
                rospy.loginfo("Published goal pose from file: x=%.2f, y=%.2f", goal_x, goal_y)

                rate.sleep()
                if rospy.is_shutdown():
                    break
    except Exception as e:
        rospy.logerr("Failed to read file: %s", str(e))

def publish_random_points(start_range, goal_range, initial_pose_pub, goal_pose_pub, rate):
    """随机生成点并发布"""
    iter_num = 0
    while not rospy.is_shutdown() and iter_num < 600:  # 限制发布次数为 1000
        # 生成随机起点
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = "world"
        initial_pose.header.stamp = rospy.Time.now()
        initial_pose.pose.pose.position.x = random.uniform(*start_range["x"])
        initial_pose.pose.pose.position.y = random.uniform(*start_range["y"])
        initial_pose.pose.pose.position.z = 0.0  # 固定 z 值
        initial_pose.pose.pose.orientation.w = 1.0  # 固定朝向

        # 生成随机终点
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "world"
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.pose.position.x = random.uniform(*goal_range["x"])
        goal_pose.pose.position.y = random.uniform(*goal_range["y"])
        goal_pose.pose.position.z = 0.0  # 固定 z 值
        goal_pose.pose.orientation.w = 1.0  # 固定朝向

        # 计算两点之间的距离
        distance = math.sqrt(
            (initial_pose.pose.pose.position.x - goal_pose.pose.position.x) ** 2 +
            (initial_pose.pose.pose.position.y - goal_pose.pose.position.y) ** 2
        )

        # 如果距离小于 50，则跳过当前循环
        if distance < 50:
            continue

        # 发布消息
        initial_pose_pub.publish(initial_pose)
        goal_pose_pub.publish(goal_pose)
        iter_num += 1

        rospy.loginfo("Published initial pose: x=%.2f, y=%.2f", initial_pose.pose.pose.position.x, initial_pose.pose.pose.position.y)
        rospy.loginfo("Published goal pose: x=%.2f, y=%.2f", goal_pose.pose.position.x, goal_pose.pose.position.y)
        rospy.loginfo("Distance between points: %.2f", distance)

        rate.sleep()

def main():
    rospy.init_node("random_pose_publisher", anonymous=True)

    # 定义发布者
    initial_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
    goal_pose_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

    # 定义范围
    # start_range = {"x": (-51.5, 49.5), "y": (-46.3, -19.2)}  # 起点范围 # Sprase场景使用
    # goal_range = {"x": (-51.5, 49.5), "y": (17, 44)}  # 终点范围
    start_range = {"x": (-61, 27), "y": (-61, 10)}  # 起点范围 # Scene6场景使用
    goal_range = {"x": (-61, 27), "y": (-61, 10)}  # 终点范围
    rospy.sleep(2)  # 等待发布者准备就绪
    rate = rospy.Rate(10)  # 5Hz

    # 设置模式
    mode = "file"  # 随机生成起点和终点
    file_path = "/home/bhrqhb/catkin_TGH_new/src/TGH_Planner/Utils/global_plan_record/Sprase/plan_pts.txt"  # 如果模式为 "file"，需要提供文件路径

    if mode == "random":
        rospy.loginfo("Publishing random points...")
        publish_random_points(start_range, goal_range, initial_pose_pub, goal_pose_pub, rate)
    elif mode == "file" and file_path:
        rospy.loginfo("Publishing points from file: %s", file_path)
        publish_from_file(file_path, initial_pose_pub, goal_pose_pub, rate)
    else:
        rospy.logerr("Invalid mode or file path not provided.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass