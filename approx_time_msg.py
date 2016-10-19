#!/usr/bin/env python

import os

import rospy
import message_filters
from sensor_msgs.msg import Image, Imu 
from approx_time_msg_extract.msg import SteeringReport, WheelSpeedReport

import numpy as np
import cv2
from cv_bridge import CvBridge

import h5py
import pickle
import gzip

bridge = CvBridge()
dataset = []

def callback(image, steering_rpt, speed_rpt):
    global dataset
    avg_speed = (speed_rpt.front_left + speed_rpt.front_right + speed_rpt.rear_left + speed_rpt.rear_right)/4

    # Timestamp info
    img_sec = image.header.stamp.secs
    img_nsec = image.header.stamp.nsecs

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

    # Steering angle
    steering_angle = steering_rpt.steering_wheel_angle

    print("steering time stamp: ", steering_rpt.header.stamp)
    #print("steering angle: ", steering_angle)
    #print("Average speed: ", avg_speed)

    # Record dataset
    tmp = [img_sec, img_nsec, steering_angle, avg_speed]
    dataset.append(tmp)
 

def my_shutdown_hook():
    global dataset
    print("in my_shutdown_hook")
    #pickle.dump(dataset, gzip.open("approx_timed_dataset.p","wb"))

    # HDF5 file preparation
    dataset_size = len(dataset)
    #f = h5py.File('approx_time_dataset.hdf5','w')
    #f_time = f.create_dataset('time_stamp', (dataset_size,2), maxshape=(None,2))
    #f_center_camera = f.create_dataset('center_camera', (dataset_size,480,640,3), maxshape=(None,480,640,3), compression="gzip")
    #f_steering_angle = f.create_dataset('steering_angle', (dataset_size,), maxshape=(None,))
    #f_avg_speed = f.create_dataset('avg_speed', (dataset_size,), maxshape=(None,))
    #f['/time_stamp'][:,0] = dataset[:][0]
    #f['/time_stamp'][:,1] = dataset[:][1]
    #f['/steering_angle'] = dataset[:][3]
    #f['/avg_speed'] = dataset[:][4]

    #f.close()



# Initialize ROS node
rospy.init_node('image_extract_node', anonymous=True)

# Subscribers
image_sub = message_filters.Subscriber('/center_camera/image_color', Image)
steering_rpt_sub = message_filters.Subscriber('/vehicle/steering_report', SteeringReport)
speed_rpt_sub = message_filters.Subscriber('/vehicle/wheel_speed_report', WheelSpeedReport)

while not rospy.is_shutdown():
    # Approximate time synchronizing
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, steering_rpt_sub, speed_rpt_sub], 1000, 0.5)
    ts.registerCallback(callback)

    rospy.spin()

pickle.dump(dataset, gzip.open("approx_timed_dataset.p","wb"))
#print(dataset[5])

rospy.on_shutdown(my_shutdown_hook)
