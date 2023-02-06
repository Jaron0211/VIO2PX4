#!/usr/bin/env python3

import sys
sys.path.append("/usr/local/lib/")

import os
os.environ["MAVLINK20"] = "1"

import rospy

from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

from visualization_msgs.msg import MarkerArray, Marker

import numpy as np
import transformations as tf
import math as m
import time

#-------------------------------------------------------------#
# Default configurations for connection to the FCU
connection_string_default = '/dev/ttyTHS0'
connection_baudrate_default = 921600
connection_timeout_sec_default = 1

# Transformation to convert different camera orientations to NED convention. Replace camera_orientation_default for your configuration.
#   0: Forward, USB port to the right
#   1: Downfacing, USB port to the right 
#   2: Forward, 45 degree tilted down, USB port to the right
#   3: Downfacing, USB port to the back
# Important note for downfacing camera: you need to tilt the vehicle's nose up a little - not flat - before you run the script, otherwise the initial yaw will be randomized, read here for more details: https://github.com/IntelRealSense/librealsense/issues/4080. Tilt the vehicle to any other sides and the yaw might not be as stable.
camera_orientation_default = 0

# https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
enable_msg_vision_position_estimate = True
vision_position_estimate_msg_hz_default = 30.0

# https://mavlink.io/en/messages/ardupilotmega.html#VISION_POSITION_DELTA
enable_msg_vision_position_delta = False
vision_position_delta_msg_hz_default = 10.0

# https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE
enable_msg_vision_speed_estimate = False
vision_speed_estimate_msg_hz_default = 30.0

# https://mavlink.io/en/messages/common.html#STATUSTEXT
enable_update_tracking_confidence_to_gcs = False
update_tracking_confidence_to_gcs_hz_default = 1.0

# Monitor user's online input via keyboard, can only be used when runs from terminal
enable_user_keyboard_input = True

# Default global position for EKF home/ origin
enable_auto_set_ekf_home = True
#22.995545011590906, 120.22339680606466
home_lat = 229955418    # Somewhere random
home_lon = 1202233069     # Somewhere random
home_alt = 30       # Somewhere random

# TODO: Taken care of by ArduPilot, so can be removed (once the handling on AP side is confirmed stable)
# In NED frame, offset from the IMU or the center of gravity to the camera's origin point
body_offset_enabled = 0
body_offset_x = 0  # In meters (m)
body_offset_y = 0  # In meters (m)
body_offset_z = 0  # In meters (m)

# Global scale factor, position x y z will be scaled up/down by this factor
scale_factor = 1.0

# Enable using yaw from compass to align north (zero degree is facing north)
compass_enabled = 0

# pose data confidence: 0x0 - Failed / 0x1 - Low / 0x2 - Medium / 0x3 - High 
pose_data_confidence_level = ('FAILED', 'Low', 'Medium', 'High')

# lock for thread synchronization
lock = threading.Lock()
mavlink_thread_should_exit = False

# default exit code is failure - a graceful termination with a
# terminate signal is possible.
exit_code = 1
#-------------------------------------------------------------#

# Data variables
data = None
prev_data = None
H_aeroRef_aeroBody = None
V_aeroRef_aeroBody = None
heading_north_yaw = None
current_confidence_level = None
current_time_us = round(time.time_ns/1000)

Odo_data = PoseStamped()
reset_counter = 1


# Rotation matrix
if camera_orientation == 0:     # Forward, USB port to the right
    H_aeroRef_T265Ref   = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)
elif camera_orientation == 1:   # Downfacing, USB port to the right
    H_aeroRef_T265Ref   = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    H_T265body_aeroBody = np.array([[0,1,0,0],[1,0,0,0],[0,0,-1,0],[0,0,0,1]])
elif camera_orientation == 2:   # 45degree forward, USB port to the right
    H_aeroRef_T265Ref   = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    H_T265body_aeroBody = (tf.euler_matrix(m.pi/4, 0, 0)).dot(np.linalg.inv(H_aeroRef_T265Ref))
elif camera_orientation == 3:   # Downfacing, USB port to the back
    H_aeroRef_T265Ref   = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    H_T265body_aeroBody = np.array([[-1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]])
else:                           # Default is facing forward, USB port to the right
    H_aeroRef_T265Ref   = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)

## Function
def odo_callback(data):
    global conn, reset_counter
    current_time_us = data.Header.stamp
    [POS_X,POS_Y,POS_Z] = data.pose.pose.point
    rpy_rad = np.array( tf.euler_from_matrix(data.pose.orientation, 'sxyz'))
    covariance  = data.pose.covariance

    conn.mav.vision_position_estimate_send(
                current_time_us,            # us Timestamp (UNIX time or time since system boot)
                POS_X,   # Global X position
                POS_Y,   # Global Y position
                POS_Z,   # Global Z position
                rpy_rad[0],	                # Roll angle
                rpy_rad[1],	                # Pitch angle
                rpy_rad[2],	                # Yaw angle
                covariance,                 # Row-major representation of pose 6x6 cross-covariance matrix
                reset_counter               # Estimate reset counter. Increment every time pose estimate jumps.
            )
    

##-----------------------------------------##
## Initialize the Mavlink
conn = mavutil.mavlink_connection(
    connection_string_default,
    autoreconnect = True,
    source_component = 1,
    baud=connection_baudrate_default,
    force_connected=True
)


service_timeout = 30
rospy.loginfo("waiting for VINS services")
try:
    rospy.wait_for_service('/vins_estimator/camera_pose', service_timeout)
except rospy.ROSException:
    rospy.loginfo("failed to connect to VINS services")

rospy.Subscriber('/vins_estimator/camera_pose',Odometry,odo_callback)
