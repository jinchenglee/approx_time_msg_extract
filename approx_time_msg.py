#!/usr/bin/env python

import os

import rospy
import message_filters
from sensor_msgs.msg import Image, Imu 
from approx_time_msg_extract.msg import SteeringReport, WheelSpeedReport

count = 0

def callback(image, steering_rpt, speed_rpt):
    global count
    global pub1, pub2, pub3

    # Ave speed
    avg_speed = (speed_rpt.front_left + speed_rpt.front_right + speed_rpt.rear_left + speed_rpt.rear_right)/4

    # Timestamp info
    img_sec = image.header.stamp.secs
    img_nsec = image.header.stamp.nsecs

    # Steering angle
    steering_angle = steering_rpt.steering_wheel_angle

    # Publish
    pub1.publish(image)
    pub2.publish(steering_rpt)
    pub3.publish(speed_rpt)

    count += 1

def my_shutdown_hook():
    print("in my_shutdown_hook")



# Initialize ROS node
rospy.init_node('image_extract_node', anonymous=True)

# Subscribers
image_sub = message_filters.Subscriber('/center_camera/image_color', Image)
steering_rpt_sub = message_filters.Subscriber('/vehicle/steering_report', SteeringReport)
speed_rpt_sub = message_filters.Subscriber('/vehicle/wheel_speed_report', WheelSpeedReport)

# Publisher
pub1 = rospy.Publisher('fwd_center_camera', Image, queue_size=1000)
pub2 = rospy.Publisher('fwd_steering_rpt', SteeringReport, queue_size=1000)
pub3 = rospy.Publisher('fwd_wheel_speed_rpt', WheelSpeedReport, queue_size=1000)

while not rospy.is_shutdown():
    # Approximate time synchronizing
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, steering_rpt_sub, speed_rpt_sub], 1000, 0.5)
    ts.registerCallback(callback)

    rospy.spin()

rospy.on_shutdown(my_shutdown_hook)
