#!/usr/bin/env python
import rospy
import sys, time
import math
import numpy as np
import tf
import roslib
from bisect import bisect_left, bisect_right

from import_data.import_data_class import *

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan, NavSatFix
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger, TriggerRequest
from tf.transformations import quaternion_from_euler

def check_pub_error(cur_pub_t, prev_pub_t, cur_log_t, prev_log_t, str, debug = True):
    diff_ts_pub = cur_pub_t - prev_pub_t
    diff_ts_log = cur_log_t - prev_log_t
    if diff_ts_log !=  0.0:
        diff_error = (diff_ts_log - diff_ts_pub)/diff_ts_log
        if abs(diff_error) < 0.05:
            if debug:
                rospy.loginfo('%s diff_ts: pub: %.6f log: %.6f error: %.3f' % (str, diff_ts_pub, diff_ts_log, diff_error))
        elif abs(diff_error) > 0.2:
            rospy.logerr('%s diff_ts: pub: %.6f log: %.6f error: %.3f' % (str, diff_ts_pub, diff_ts_log, diff_error))
        else:
            rospy.logwarn('%s diff_ts: pub: %.6f log: %.6f error: %.3f' % (str, diff_ts_pub, diff_ts_log, diff_error))

def talker():
	debug = rospy.get_param('load_collections/debug', False)
	# Set data_dir and exp in fpn_setup/yaml/paths_###.yaml file
	data_dir = rospy.get_param('data_dir')
	
	rospy.loginfo('data_dir: %s', data_dir)

	# Get params
	exp = rospy.get_param('exp')
	rospy.loginfo('Exp: %s' % exp)
	
	# Import datalogs
	logs = ImportedData(data_dir)
	pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
	pub_scan = rospy.Publisher('scan', LaserScan, queue_size=10)
	pub_fix = rospy.Publisher('fix', NavSatFix, queue_size=1)
	pub_imu = rospy.Publisher('imu', Imu, queue_size=1)
	pub_mhe = rospy.Publisher('mhe', Odometry, queue_size=10)
	pub_pl = rospy.Publisher('log_pl', Float32MultiArray, queue_size=1)
	br = tf.TransformBroadcaster()

	if exp in list(logs.lidar):
		lidar_ts = list(logs.lidar[exp])
		lidar = logs.lidar[exp]

	if exp in list(logs.datalog):				
		try:
			system_log_ts = logs.datalog[exp]['    (1)uptime  (ms)'].copy()
		except:
			system_log_ts = logs.datalog[exp]['     (1)uptime (ms)'].copy()
		
		ax = logs.datalog[exp]['(26)accelerometer x (m/s^2)'].copy()
		lat = logs.datalog[exp]['(22)primary gps latitude (deg)'].copy()
		lon = logs.datalog[exp]['(23)primary gps longitude (deg)'].copy()
		
		try:
			mhe_x = logs.datalog[exp]['(14)mhe output x (m)'].copy()
			mhe_y = logs.datalog[exp]['(15)mhe output y (m)'].copy()
		except:
			mhe_x = logs.datalog[exp]['(14)mhe output x  (m)'].copy()
			mhe_y = logs.datalog[exp]['(15)mhe output y  (m)'].copy()
		
		vl = logs.datalog[exp]['(40)speed calculation from encoder left (m/s)'].copy()
		vr = logs.datalog[exp]['(41)speed calculation from encoder right (m/s)'].copy()
		wz = logs.datalog[exp]['(29)gyro yaw rate (rad/s)'].copy()
		_wz = logs.datalog[exp]['(29)gyro yaw rate (rad/s)'].copy()

		aux = 2
		for i in range(aux, len(wz)-aux):
			arr = _wz[i-aux:i+aux]
			wz[i] = np.median(arr)

		
		if exp in list(logs.pTS):
			pl_log_ts = logs.pTS[exp]['timestamp']
			
			try:
				slope_l = logs.pTS[exp]['slope_l']
				slope_r = logs.pTS[exp]['slope_r']
			except:
				slope_l = logs.pTS[exp]['sl']
				slope_r = logs.pTS[exp]['sr']
			
			dl = logs.pTS[exp]['distance_left']
			dr = logs.pTS[exp]['distance_right']
			valid_l = logs.pTS[exp]['valid_l']
			valid_r = logs.pTS[exp]['valid_r']

	th = 0
	x = 0
	y = 0

	last_lidar_idx = -1
	last_pl_log_idx = -1
	last_system_log_idx = -1
	rospy.set_param('is_lidar_ready', True)

	while not rospy.is_shutdown():
		# Get simulation timestamp
		timestamp = rospy.get_time() * 1000
		
		'''
		Lidar Datalog to LidarScan message:
		'''
		# Check which lidar idx corresponds to this timestamp
		lidar_idx = bisect_left(lidar_ts, timestamp)

		# Check if it's time to read a new lidar idx, to avoid publish repeated indexes
		if last_lidar_idx < lidar_idx and lidar_idx < len(lidar_ts):
			last_lidar_idx = lidar_idx
		
			scan = LaserScan()
			scan.header.stamp = rospy.Time.now()
			scan.header.frame_id = '/base_link'
			scan.angle_min = -0.75*math.pi
			scan.angle_max = 0.75*math.pi
			scan.angle_increment = 0.25*math.pi/180.0
			scan.range_max = 30.0
			l = lidar[lidar_ts[lidar_idx]]
			l = 0.001*np.array(l)
			scan.ranges = l
			pub_scan.publish(scan)

			#check_pub_error(timestamp, prev_scan_t, 0.001*lidar_ts[lidar_idx], 0.001*prev_lidar_ts, 'SCAN', debug)

		'''
		Perception Datalog to Float32MultiArray message:
		'''
		# Check which perception_lidar idx corresponds to this timestamp
		pl_log_idx = bisect_left(pl_log_ts, timestamp)

		# Check if it's time to read a new perception data, to avoid publish repeated data
		if last_pl_log_idx < pl_log_idx and pl_log_idx < len(pl_log_ts):
			last_pl_log_idx = pl_log_idx

			vec = Float32MultiArray()

			rospy.loginfo('Perception log index: %d' % pl_log_idx)

			vec.data = [dl[pl_log_idx], dr[pl_log_idx], 
						math.atan(slope_l[pl_log_idx]),
						math.atan(slope_r[pl_log_idx]), 
						valid_l[pl_log_idx],
						valid_r[pl_log_idx]]

			pub_pl.publish(vec)
			#check_pub_error(cur_t, prev_pl_pub_t, 0.001*pl_log_ts[pl_log_idx], 0.001*prev_pl_log_ts, 'PL_LOG', debug  )

		'''
		System Datalog to Odometry, NavSatFix and Imu messages:
			- Encoder
			- GPS
			- IMU
			- MHE
		'''
		# Check which perception_lidar idx corresponds to this timestamp
		system_log_idx = bisect_left(system_log_ts, timestamp)

		# Check if it's time to read a new system log data, to avoid publish repeated data
		if last_system_log_idx < system_log_idx and system_log_idx < len(system_log_ts):
			last_system_log_idx = system_log_idx

			vx = (vl[system_log_idx] + vr[system_log_idx]) * 0.5
			_wz = wz[system_log_idx]
			
			#check_pub_error(cur_t, prev_system_pub_t, 0.001*system_log_ts[system_log_idx], 0.001*prev_system_log_ts, 'SYSTEM_LOG', debug  )
			
			if(system_log_idx > 0):
				dt = (system_log_ts[system_log_idx] - system_log_ts[system_log_idx-1])/1000.0
			else:
				dt = 0
	
			odom = Odometry()

			th += _wz*dt
			x += vx*math.cos(th)*dt
			y += vx*math.sin(th)*dt
			q = quaternion_from_euler(0.0, 0.0, th)
			br.sendTransform((x, y, 0.0), q, rospy.Time.now(), "/base_link", "/map")
			odom.pose.pose.position.x = x
			odom.pose.pose.position.y = y
			odom.pose.pose.position.x
			odom.pose.pose.orientation.x = q[0]
			odom.pose.pose.orientation.y = q[1]
			odom.pose.pose.orientation.z = q[2]
			odom.pose.pose.orientation.w = q[3]
			odom.twist.twist.linear.x = vx
			odom.twist.twist.angular.z = _wz
			odom.header.stamp = rospy.Time.now()
			odom.header.frame_id = "/map"
			odom.child_frame_id = "/base_link"

			fix = NavSatFix()
			fix.header.stamp = rospy.Time.now()
			fix.latitude = lat[system_log_idx]
			fix.longitude = lon[system_log_idx]

			imu = Imu()
			imu.angular_velocity.z = ax[system_log_idx]
			imu.linear_acceleration.x = wz[system_log_idx]
			imu.header.stamp = rospy.Time.now()

			mhe = Odometry()
			mhe.pose.pose.position.x = mhe_x[system_log_idx]
			mhe.pose.pose.position.y = mhe_y[system_log_idx]
			mhe.header.stamp = rospy.Time.now()
			mhe.header.frame_id = "/map"
			mhe.child_frame_id = "/base_link"

			pub_fix.publish(fix)
			pub_imu.publish(imu)
			pub_mhe.publish(mhe)
			pub_odom.publish(odom)

if __name__ == '__main__':
    rospy.init_node('datalog_talker', anonymous=True)

    try:
        talker()
    except rospy.ROSInterruptException:
        pass
      