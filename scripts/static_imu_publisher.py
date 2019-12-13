#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

def main():
    rospy.init_node('fake_imu_publisher')

    pub = rospy.Publisher('imu', Imu, queue_size=10)

    def odom_subscriber_cb(data):
        imu_msg = Imu()
        imu_msg.header = data.header
        imu_msg.header.frame_id = 'base_link'
        imu_msg.orientation = data.pose.pose.orientation
        imu_msg.linear_acceleration.z = -10
        pub.publish(imu_msg)

    odom_subscriber = rospy.Subscriber("odometry", Odometry, odom_subscriber_cb, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass