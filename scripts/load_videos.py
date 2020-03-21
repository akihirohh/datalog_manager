#!/usr/bin/env python
import rospy
import cv2
from bisect import bisect_left, bisect_right
from cv_bridge import CvBridge, CvBridgeError

# ROS messages
from sensor_msgs.msg import Image, CameraInfo

# Custom libraries
from import_data.import_data_class import *

def talker():
	# Get frame_id from rosparam
	frame_id = rospy.get_param('frame_id', 'camera')
	# Start rosnode
	rospy.init_node('datalog_video_talker', anonymous=True)
    
	bridge = CvBridge()

	# Set data_dir and exps in fpn_setup/yaml/paths_###.yaml file
	data_dir = rospy.get_param('data_dir')
	exp = rospy.get_param('exp')
	logs = ImportedData(data_dir, import_lidar=False)
	
	img_pub = rospy.Publisher('front_cam/raw_image', Image, queue_size=10)

	if exp in list(logs.cam_front_fp):
		video_filepath = logs.cam_front_fp[exp]
		front_cam_ts = logs.cam_front[exp].capture_time_ms
		front_cap = cv2.VideoCapture(str(video_filepath))
	else:
		rospy.signal_shutdown('No exp')
		pass

	while(rospy.get_param('is_lidar_ready') is not True):
			#rospy.loginfo('waiting for lidar data')
			pass

	num_frames = int(front_cap.get(cv2.CAP_PROP_FRAME_COUNT))

	last_front_frame = -1
	while not rospy.is_shutdown():
		# Get simulation timestamp
		timestamp = rospy.get_time() * 1000
		# Check which camera frame corresponds to this timestamp
		front_cam_frame = bisect_left(front_cam_ts, timestamp)

		# Check if it's time to read a new camera frame, to avoid publish repeated frames
		if last_front_frame < front_cam_frame and front_cam_frame < num_frames:
			last_front_frame = front_cam_frame
			front_cap.set(cv2.CAP_PROP_POS_FRAMES, front_cam_frame)
			success, image = front_cap.read()
			
			if success:
				# Publish image.
				img_msg = bridge.cv2_to_imgmsg(image, "bgr8")
				img_msg.header.stamp = rospy.Time.now()
				img_msg.header.frame_id = frame_id
				
				img_pub.publish(img_msg)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
        