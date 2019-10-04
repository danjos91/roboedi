#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

t_range = 1

# def move(t_range):
#    t0 = rospy.Time.now().to_sec()
#    current_distance = 0
#
#    while (current_distance < t_range):



def callback(msg):
    global t_range
    t_range = msg.ranges[0]
    if (t_range > 0.34):
        vel_msg.linear.x = 0.05
        vel_pub.publish(vel_msg)
    else:
        vel_msg.linear.x = 0
        vel_pub.publish(vel_msg)

rospy.init_node('robot_scanner', anonymous=True)

vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
vel_msg = Twist()

scan_sub = rospy.Subscriber('/scan', LaserScan, callback)

while (t_range > 0.34):
    print(t_range)
