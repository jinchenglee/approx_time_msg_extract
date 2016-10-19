#!/usr/bin/env python

import os

import rospy
import message_filters
from sensor_msgs.msg import Image, Imu 
from approx_time_msg_extract.msg import SteeringReport, WheelSpeedReport

import numpy as np
import cv2
from cv_bridge import CvBridge

import tables

bridge = CvBridge()
count = 0
img_sec_list = []
img_nsec_list = []
steering_angle_list = []
avg_speed_list = []

def callback(image, steering_rpt, speed_rpt):
    global count
    global f 
    global filters 
    global f_img_sec
    global f_img_nsec
    global f_steering_angle
    global f_avg_speed
    global img_sec_list
    global img_nsec_list
    global steering_angle_list
    global avg_speed_list

    # Ave speed
    avg_speed = (speed_rpt.front_left + speed_rpt.front_right + speed_rpt.rear_left + speed_rpt.rear_right)/4

    # Timestamp info
    img_sec = image.header.stamp.secs
    img_nsec = image.header.stamp.nsecs

    # Steering angle
    steering_angle = steering_rpt.steering_wheel_angle

    # Save to list
    img_sec_list.append(img_sec)
    img_nsec_list.append(img_nsec)
    steering_angle_list.append(steering_angle)
    avg_speed_list.append(avg_speed)

    ## Get image from message packet
    #cv_img = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")

    ## Crop camera input image
    #cv_img_cropped = cv_img[300:479,:,:]

    ## Convert to grayscale
    #cv_img_cropped_grayscale = cv2.cvtColor(cv_img_cropped, cv2.COLOR_BGR2GRAY)

    ## Luminance average
    #cv_img_cropped_grayscale_avg = cv_img_cropped_grayscale.mean()
    #print("img average luma {0}".format(cv_img_cropped_grayscale_avg))

    # Draw a line. line(img, start_pxl, end_pxl, color, width)
    #cv2.line(cv_img_cropped_grayscale,(240,20), (70,50),(255,255,255),1)

    # Save to image file
    #file_name = "./frame" + str(image.header.stamp.secs) + "." + str(image.header.stamp.nsecs) + ".png"
    #print("File name = ", file_name)
    #cv2.imwrite(os.path.join(args.output_dir, "frame%06i.png" % count), cv_img_cropped_grayscale)
    #cv2.imwrite(file_name, cv_img_cropped_grayscale)

    # Record dataset
    count += 1

    if count%1000 == 0:
        print("Time stamp: ", img_sec, img_nsec, "count: ", count)
        f_img_sec.append(img_sec_list)
        f_img_nsec.append(img_nsec_list)
        f_steering_angle.append(steering_angle_list)
        f_avg_speed.append(avg_speed_list)
        img_sec_list = []
        img_nsec_list = []
        steering_angle_list = []
        avg_speed_list = []



def my_shutdown_hook():
    global dataset
    print("in my_shutdown_hook")

# Initialize ROS node
rospy.init_node('image_extract_node', anonymous=True)

# Subscribers
image_sub = message_filters.Subscriber('/center_camera/image_color', Image)
steering_rpt_sub = message_filters.Subscriber('/vehicle/steering_report', SteeringReport)
speed_rpt_sub = message_filters.Subscriber('/vehicle/wheel_speed_report', WheelSpeedReport)

# PyTable file
hdf5_path = "approx_timed_time_steering_speed.hdf5"
f = tables.open_file(hdf5_path,'w')
filters = tables.Filters(complevel=5, complib='blosc')
f_img_sec = f.create_earray(f.root, 'img_sec', tables.Atom.from_dtype(np.dtype('int')), shape=(0,), filters=filters)
f_img_nsec = f.create_earray(f.root, 'img_nsec', tables.Atom.from_dtype(np.dtype('int')), shape=(0,), filters=filters)
f_steering_angle = f.create_earray(f.root, 'steering_angle', tables.Atom.from_dtype(np.dtype('float')), shape=(0,), filters=filters)
f_avg_speed = f.create_earray(f.root, 'avg_speed', tables.Atom.from_dtype(np.dtype('float')), shape=(0,), filters=filters)


while not rospy.is_shutdown():
    # Approximate time synchronizing
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, steering_rpt_sub, speed_rpt_sub], 1000, 0.5)
    ts.registerCallback(callback)

    rospy.spin()

print("Exiting... Total count: ", count)
f_img_sec.append(img_sec_list)
f_img_nsec.append(img_nsec_list)
f_steering_angle.append(steering_angle_list)
f_avg_speed.append(avg_speed_list)
img_sec_list = []
img_nsec_list = []
steering_angle_list = []
avg_speed_list = []

f.close()

rospy.on_shutdown(my_shutdown_hook)
