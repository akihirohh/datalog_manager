# -*- coding: utf-8 -*-


# Folder names
bags_folder = 'bags'
lidar_folder = 'lidar'
system_folder = 'datalog'

# File names
datafile_name = '*data.txt'
lidar_fn = 'lidar_log'
secondary_lidar_fn = 'secondary_lidar_log'

left_filteredScan_fn = 'left_filteredScan'
right_filteredScan_fn = 'right_filteredScan'
pretend_lidar_fn = 'pretend_lidar.txt'
pTS_fn = 'perception_lidar_log'
system_fn = 'system_log'
        
# filename : [ImportedData object name, num of header lines in file]
# the specified num of header lines is not read, therefore do not
# count the header line with the variable names for the dataset
specific_logs = {'datalog', \
                'lidar', \
                'secondary_lidar', \
                'left_lidar', \
                'right_lidar', \
                'pretend_lidar', \
                'pTS', \
                'pTS_configs', \
                'name', \
                'bags', \
                'bags_aux', \
                'modify_time'
                }

general_logs = {'ps_ransac.txt' : ['pRansac',0], \
                'hinf_out.txt' : ['hinf',0],\
                'data.txt' : ['data',0], \
                'perception_jfr' : ['pJFR',0], \
                'perception_log_new' : ['pN',1], \
                'pretend_perception.txt' : ['pP',1], \
                'gps.txt' : ['gps', 0], \
                'ps_recorded.txt' : ['pR', 0], \
                'perception_lidar_log' : ['pTS', 1], \
                'extended_perception_lidar_log' : ['epTS', 0], \
                'left_ps' : ['left_ps', 1], \
                'right_ps' : ['right_ps', 1]
                }
cam_logs = {'cam_right' : 'cam_right', \
            'cam_left' : 'cam_left', \
            'cam_front' : 'cam_front', \
            'cam_down' : 'cam_down'
            }
cam_filepath = {'cam_right' : 'cam_right_fp', \
            'cam_left' : 'cam_left_fp', \
            'cam_front' : 'cam_front_fp', \
            'cam_down' : 'cam_down_fp'
            } 

xy_logs = {'inliers.txt' : 'inliers', \
           'inliers_l_log' : 'inliersL', \
           'inliers_r_log' : 'inliersR', \
           'chosen_l' : 'chosen_l', \
           'chosen_r' : 'chosen_r', \
           'split_l' : 'split_l', \
           'split_r' : 'split_r'}

cam_dnames=['capture_time_ms',
            'capture_period_ms',
            'capture_latency_ms',
            'process_latency_ms',
            'stream_latency_ms',
            'record_latency_ms']   
  
sys_dnames = ["linux_time_ms",
              "uptime_ms",
              "drive_mode",
              "mpc_step_counter",
              "mpc_origin_latitude",
              "mpc_origin_longitude__deg",
              "closest_trajectory_point_index_to_robot",
              "mpc_error_x_W_E_m",
              "mpc_error_y_W_E_m",
              "abs_mpc_error_m",
              "input_x_for_mhe_m",
              "input_y_for_mhe_m",
              "input_speed_for_mhe_m_s",
              "input_vyaw_for_mhe_rad_s",
              "mhe_output_x_m",
              "mhe_output_y_m",
              "mhe_output_speed_m_s",
              "mhe_output_yaw_rad",
              "mpc_reference_heading_rad",
              "mpc_turn_rate_command_rad_s",
              "turn_rate_command_as_PWM",
              "throttle_command_as_PWM",
              "primary_gps_latitude_deg",
              "primary_gps_longitude_deg",
              "secondary_gps_latitude_deg",
              "secondary_gps_longitude_deg",
              "accelerometer_x_m_s2",
              "accelerometer_y_m_s2",
              "accelerometer_z_m_s2",
              "gyro_yaw_rate_rad_s",
              "gyro_pitch_rate_rad_s",
              "gyro_roll_rate_rad_s",
              "yaw_rad",
              "pitch_rad",
              "roll_rad",
              "sonar1_measurement_m",
              "ultrasonic_sensor_measurement_m",
              "primary_gps_time_s",
              "normalized_turn_rate_command",
              "normalized_throttle_command",
              "speed_calculation_from_encoder_left_m_s",
              "speed_calculation_from_encoder_right_m_s",
              "front_gimbal_target_angle_degrees_from_straight_down",
              "left_gimbal_target_angle_degrees_from_straight_down",
              "right_gimbal_target_angle_degrees_from_straight_down",
              "front_gimbal_measured_angle_degrees_from_straight_down",
              "left_gimbal_measured_angle_degrees_from_straight_down",
              "right_gimbal_measured_angle_degrees_from_straight_down",
              "internal_temperature_C",
              "battery_voltage_V",
              "primary_gps_type",
              "secondary_gps_type",
              "MPC_R_value",
              "MPC_Q0_value",
              "streaming_sensors_value",
              "front_gimbal_yaw",
              "front_gimbal_pitch",
              "front_gimbal_roll",
              "left_gimbal_yaw",
              "left_gimbal_pitch",
              "left_gimbal_roll",
              "right_gimbal_yaw",
              "right_gimbal_pitch",
              "right_gimbal_roll",
              "ekf_estimated_x",
              "ekf_estimated_y",
              "ekf_estimated_theta",
              "gps_speed",
              "gps_satellites",
              "sec_satellites",
              "latest_motion_cmd_in_throttle_percent",
              "latest_motion_cmd_in_yaw_percent",
              "latest_motion_cmd_out_throttle",
              "latest_motion_cmd_out_yaw_percent",
              "mhe_m",
              "mhe_traction_coef",

              "aux","aux2","aux3","aux4","aux5"]