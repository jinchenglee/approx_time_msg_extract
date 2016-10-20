# approx_time_msg_extract
ROS message extraction and publisher with approximate time synchronization.

----------------------
How to compile to get a ROS package in place
----------------------
First, follow this tutorial: http://wiki.ros.org/ROS/Tutorials/catkin/CreatingPackage

You should have your catkin workspace similar like this:

catkin_ws
   
   ├──  devel
   
   ├──  build
   
   └──  src
      
          └── approx_time_msg_extract     <= this repository

Then, 
  > cd catkin_ws

  > catkin_make

----------------------
How to run the script?
----------------------

1. Start roscore.
    > roscore

2. Play back a .bag file.
    > rosbag play foo.bag

3. In yet another terminal, start recording on certain messages we care. 
    > rosbag record /fwd_center_camera /fwd_steering_rpt /fwd_wheel_speed_rpt

4. Launch the message extract (with approximate time synchronization) and forward script.
    > rosrun approx_time_msg_extract approx_time_msg.py

If everything works fine, using > rqt_graph should show a connect diagram like below. 
![alt tag](https://github.com/jinchenglee/approx_time_msg_extract/blob/master/ros_play_extract_conn.png)

----------------------
Convert the time-sync'ed .bag file to PyTables hdf5 file
----------------------
This bag2hdf5.py script is stand-alone, not required to be part of this ROS package. But it is very specifically written to extract the "/fwd_center_camera /fwd_steering_rpt /fwd_wheel_speed_rpt" topics though. 

> python bag2hdf5.py file.bag



