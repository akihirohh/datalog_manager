#!/usr/bin/env python

import rospy
import sys, time

# ROS messages
from rosgraph_msgs.msg import Clock

# Custom libraries
from import_data.import_data_class import *

def clock():
    clock_pub = rospy.Publisher('/clock', Clock, queue_size=10)
    rospy.init_node('datalog_clock_node')
    
    # Get Params
    clock_loop = rospy.get_param('clock_loop', False)

    # Set data_dir and exps in fpn_setup/yaml/paths_###.yaml file
    data_dir = rospy.get_param('data_dir')
    exp = rospy.get_param('exp')
  
    rate = 1000 # Set loop time to 1 kHz

    # Read datalogs
    logs = ImportedData(data_dir, import_lidar=False)

    # Verifies if datalog is empty
    if exp in list(logs.datalog):            
        # Get first datalog timestamp
        try:
            datalog_init_time = logs.datalog[exp]['    (1)uptime  (ms)'][0]
            datalog_final_time = logs.datalog[exp]['    (1)uptime  (ms)'][0]
        except:
            datalog_init_time = logs.datalog[exp]['     (1)uptime (ms)'][0]
            datalog_final_time = logs.datalog[exp]['     (1)uptime (ms)'][0]

    # Transform it in seconds
    datalog_init_time = datalog_init_time/1000.0
    datalog_final_time = datalog_final_time/1000.0

    # Get initial system time
    init_sys_time = time.time()
    sim_time = datalog_init_time

    while(rospy.get_param('is_lidar_ready') is not True):
        #rospy.loginfo('waiting for lidar data')
        pass

    while not rospy.is_shutdown():
        if clock_loop and (sim_time > datalog_final_time):
            init_sys_time = time.time()
        # Compare system time with datalog time
        init_loop_time = time.time()
        delta_time = init_loop_time - init_sys_time
        sim_time = datalog_init_time + delta_time
        t = rospy.Time.from_sec(sim_time)

        clock_msg = Clock()
        clock_msg.clock = t

        clock_pub.publish(clock_msg)

        # Because we are publishing \clock, we cannot use rospy.sleep()
        while (time.time()-init_loop_time) < 1/rate:
            if rospy.is_shutdown(): # hopefully, this will shut down the node every time when using datalog
                break
            pass
    

if __name__ == '__main__':
  try:
    clock()
  except rospy.ROSInterruptException:
    pass