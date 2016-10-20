#!/usr/bin/env python

"""Extract images/steering angle/speed from a time-synchronized rosbag to a PyTable hdf5 file.
"""
 
import os
import argparse
 
import cv2
from cv_bridge import CvBridge
import numpy as np
 
import rosbag
from sensor_msgs.msg import Image

import tables

IMG_W = 640
IMG_H = 180
DATA_NUM = 3000

#def dump_img():
#    f = tables.open_file('./dataset_converted.hdf5','r')
#    dset = f.root.center_camera
#    for i in range(2068):
#        cv2.imwrite("./tmp/frame%04i.png" % i, dset[i,:,:,:])

def main():
    parser = argparse.ArgumentParser(description="Extract images/steering angle/speed from a time-synchronized rosbag to a PyTable hdf5 file.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    #parser.add_argument("output_dir", help="Output directory.")
    #parser.add_argument("image_topic", help="Image topic.")
 
    args = parser.parse_args()
 
    #print( "Extract images from %s on topic %s into %s" % (args.bag_file,
    #                                                      args.image_topic, args.output_dir))
    bag = rosbag.Bag(args.bag_file, "r")
    bridge = CvBridge()

    count = 0
    cv_img_cropped_list=[]
    img_sec_list = []
    img_nsec_list = []
    avg_speed_list = []
    steering_angle_list = []

    tmp_im = np.ndarray(shape=(IMG_H, IMG_W, 3))
    message = bridge.cv2_to_imgmsg(tmp_im) 
    cv_img_cropped = bridge.imgmsg_to_cv2(message)
    print("cv_img_cropped shape: ", cv_img_cropped.shape)

    # PyTable hdf5 storage file
    hdf5_path = "./dataset_converted.hdf5"
    f = tables.open_file(hdf5_path,'w')
    filters = tables.Filters(complevel=5, complib='blosc')

    ## ------------------------------------
    ## Ugly hack only to get cv_img_cropped.dtype for PyTable atom data type.
    ## ------------------------------------
    #for topic, msg, t in bag.read_messages():
    #    if topic == 'sensor_msgs/Image':
    #        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    #        cv_img_cropped = cv_img[300:480,:,:]
    #        print("cv_img_cropped shape: ", cv_img_cropped.shape)
 
    #        break;

    # EArrays of PyTable
    f_center_camera = f.create_earray(f.root, 'center_camera', tables.Atom.from_dtype(cv_img_cropped.dtype), shape=(0,IMG_H,IMG_W,3), filters=filters, expectedrows=len(cv_img_cropped))
    f_img_sec = f.create_earray(f.root, 'img_sec', tables.Atom.from_dtype(np.dtype('int')), shape=(0,), filters=filters)
    f_img_nsec = f.create_earray(f.root, 'img_nsec', tables.Atom.from_dtype(np.dtype('int')), shape=(0,), filters=filters)
    f_steering_angle = f.create_earray(f.root, 'steering_angle', tables.Atom.from_dtype(np.dtype('float')), shape=(0,), filters=filters)
    f_avg_speed = f.create_earray(f.root, 'avg_speed', tables.Atom.from_dtype(np.dtype('float')), shape=(0,), filters=filters)

    # --------------------------------------
    # Processing the bag file -- officially.
    # --------------------------------------
    #for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
    for topic, msg, t in bag.read_messages():
        if topic == '/fwd_center_camera':

            img_sec = msg.header.stamp.secs
            img_nsec = msg.header.stamp.nsecs
            img_sec_list.append(img_sec)
            img_nsec_list.append(img_nsec)

            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            cv_img_cropped = cv_img[300:480,:,:]
            cv_img_cropped_list.append(cv_img_cropped)
            #cv_img_cropped_grayscale = cv2.cvtColor(cv_img_cropped, cv2.COLOR_BGR2GRAY)
            # Draw a line. line(img, start_pxl, end_pxl, color, width)
            #cv2.line(cv_img_cropped_grayscale,(240,20), (70,50),(255,255,255),1)

            count += 1

        elif topic == '/fwd_wheel_speed_rpt':

            avg_speed = (msg.front_left + msg.front_right + msg.rear_left + msg.rear_right)/4
            avg_speed_list.append(avg_speed)

            count += 1

        elif topic == '/fwd_steering_rpt':

            steering_angle = msg.steering_wheel_angle
            steering_angle_list.append(steering_angle)

            count += 1

        if ((count % DATA_NUM) == 0) and (count != 0):
            print("Message count: ", count)
            f_center_camera.append(cv_img_cropped_list)
            f_img_sec.append(img_sec_list)
            f_img_nsec.append(img_nsec_list)
            f_steering_angle.append(steering_angle_list)
            f_avg_speed.append(avg_speed_list)

            cv_img_cropped_list=[]
            img_sec_list = []
            img_nsec_list = []
            steering_angle_list = []
            avg_speed_list = []

    if count != 0:
        print("Message count (last): ", count)
        f_center_camera.append(cv_img_cropped_list)
        f_img_sec.append(img_sec_list)
        f_img_nsec.append(img_nsec_list)
        f_steering_angle.append(steering_angle_list)
        f_avg_speed.append(avg_speed_list)

        cv_img_cropped_list=[]
        img_sec_list = []
        img_nsec_list = []
        steering_angle_list = []
        avg_speed_list = []

    bag.close()
    f.close()
 
    return
 
if __name__ == '__main__':
    main()
