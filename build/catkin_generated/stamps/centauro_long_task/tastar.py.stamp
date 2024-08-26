#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import std_msgs.msg
import struct
import random
import time

def create_test_pointcloud2():
    """
    创建一个简单的 PointCloud2 消息用于测试
    """
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]

    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'

    points = []
    for i in range(10):  # 生成10个随机点
        x = random.uniform(-5, 5)
        y = random.uniform(-5, 5)
        z = random.uniform(-1, 3)
        points.append([x, y, z])

    # 将点数据转换为二进制格式
    buffer = []
    for p in points:
        buffer.append(struct.pack('fff', *p))
    
    buffer = b''.join(buffer)

    pointcloud = PointCloud2(
        header=header,
        height=1,
        width=len(points),
        is_dense=True,
        is_bigendian=False,
        fields=fields,
        point_step=12,
        row_step=12 * len(points),
        data=buffer
    )
    return pointcloud

def create_test_posestamped(frame_id, x, y, z):
    """
    创建一个简单的 PoseStamped 消息用于测试
    """
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = Quaternion(x=0, y=0, z=0, w=1)

    pose_stamped = PoseStamped(header=header, pose=pose)
    return pose_stamped

def main():
    rospy.init_node('test_publisher', anonymous=True)

    map_pub = rospy.Publisher("/map", PointCloud2, queue_size=1)
    start_pub = rospy.Publisher("/start_point", PoseStamped, queue_size=10)
    goal_pub = rospy.Publisher("/goal", PoseStamped, queue_size=10)

    rate = rospy.Rate(1)  # 1Hz

    # while not rospy.is_shutdown():
        # 发布测试 PointCloud2 消息
    rospy.sleep(2)
    pointcloud = create_test_pointcloud2()
    map_pub.publish(pointcloud)
    rospy.loginfo("Published test PointCloud2")
    rospy.sleep(2)
    # 发布测试起点 PoseStamped 消息
    start_pose = create_test_posestamped("map", 0.0, 0.0, 0.0)
    start_pub.publish(start_pose)
    rospy.loginfo("Published test start PoseStamped")
    rospy.sleep(4)
    # 发布测试终点 PoseStamped 消息
    goal_pose = create_test_posestamped("map", 10.0, 5.0, 0.0)
    goal_pub.publish(goal_pose)
    rospy.loginfo("Published test goal PoseStamped")

    rospy.sleep(2)
    pointcloud = create_test_pointcloud2()
    map_pub.publish(pointcloud)
    rospy.loginfo("again PointCloud2")
    rospy.sleep(2)
    # 发布测试起点 PoseStamped 消息
    start_pose = create_test_posestamped("map", 10.0, 5.0, 0.0)
    start_pub.publish(start_pose)
    rospy.loginfo("Published test start PoseStamped")
    rospy.sleep(4)
    # 发布测试终点 PoseStamped 消息
    goal_pose = create_test_posestamped("map", 25.0, -7.0, 0.0)
    goal_pub.publish(goal_pose)
    rospy.loginfo("Published test goal PoseStamped")
    rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
