#!/usr/bin/env python3

import sys
sys.path.append("/usr/local/lib/")

import os
os.environ["MAVLINK20"] = "1"

import rospy
from pymavlink import mavutil

from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

from visualization_msgs.msg import MarkerArray, Marker

import numpy as np
import transformations as tf
import math as m
import time
import threading

#-------------------------------------------------------------#
# Default configurations for connection to the FCU
connection_string = '/dev/ttyTHS0'
connection_baudrate = 921600
connection_timeout_sec = 1

# Transformation to convert different camera orientations to NED convention. Replace camera_orientation_default for your configuration.
#   0: Forward, USB port to the right
#   1: Downfacing, USB port to the right 
#   2: Forward, 45 degree tilted down, USB port to the right
#   3: Downfacing, USB port to the back
# Important note for downfacing camera: you need to tilt the vehicle's nose up a little - not flat - before you run the script, otherwise the initial yaw will be randomized, read here for more details: https://github.com/IntelRealSense/librealsense/issues/4080. Tilt the vehicle to any other sides and the yaw might not be as stable.
camera_orientation = 0

# https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
enable_msg_vision_position_estimate = True
vision_position_estimate_msg_hz = 30.0

# https://mavlink.io/en/messages/ardupilotmega.html#VISION_POSITION_DELTA
enable_msg_vision_position_delta = False
vision_position_delta_msg_hz = 10.0

# https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE
enable_msg_vision_speed_estimate = False
vision_speed_estimate_msg_hz = 30.0

# https://mavlink.io/en/messages/common.html#STATUSTEXT
enable_update_tracking_confidence_to_gcs = False
update_tracking_confidence_to_gcs_hz = 1.0

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
current_time_us = round(time.time_ns()/1000)

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
import math
 
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
timer = time.time_ns

def odo_callback(data):
    global conn, reset_counter
    current_time_us = timer()
    POS_X = data.pose.pose.position.x
    POS_Y = data.pose.pose.position.y
    POS_Z = data.pose.pose.position.z
    
    ori_x = data.pose.pose.orientation.x
    ori_y = data.pose.pose.orientation.y
    ori_z = data.pose.pose.orientation.z
    ori_w = data.pose.pose.orientation.w

    rpy_rad = euler_from_quaternion(ori_x,ori_y,ori_z,ori_w)
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
    connection_string,
    autoreconnect = True,
    source_component = 1,
    baud=connection_baudrate,
    force_connected=True
)

rospy.init_node('vin2pix')
rospy.loginfo("waiting for VINS services")

rospy.Subscriber('/vins_estimator/odometry',Odometry,odo_callback)
print('OK')
rospy.spin()
