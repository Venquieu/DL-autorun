import rospy
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge
import cv2
import math
import copy
import defaults
from aw_idl.msg import LocalizedPose

# Instantiate CvBridge
pose_topic = "/aw/localized_pose"
bridge = CvBridge()

is_stationary = False
start_timestamp = None
speed_thr = defaults.stationary_speed_thres
mobile_speed_thr = defaults.mobile_speed_thres
contiguous_length = defaults.stationary_contiguous_length
# min_duration_time = defaults.stationary_min_duration_time

velo_pre = [10]*(contiguous_length - 1)
time_point_list = []

def is_stationary_before():
    return max(velo_pre) < speed_thr
    
def is_mobile_before():
    return min(velo_pre) > mobile_speed_thr

def calculate_velo(velo_east,velo_north):
    return math.sqrt(velo_east**2 + velo_north**2)

def updata_velo_pre(velo_now:int):
    global velo_pre
    for i in range(contiguous_length-2, 0, -1):
        velo_pre[i] = velo_pre[i-1]
    velo_pre[0] = velo_now

def print_list2str(list_:list):
    str_list = []
    for num in list_:
        str_list.append(str(num))
    print(' '.join(str_list))

def time_list_append(stamp):
    global time_point_list
    # if len(time_point_list) == 0:
    #     time_point_list.append(stamp)
    #     return 
    # if stamp - time_point_list[-1] < min_duration_time:
    #     time_point_list.pop()
    #     return
    time_point_list.append(stamp)
    print_list2str(time_point_list)

def pose_callback(msg):
    global is_stationary
    global start_timestamp
    global velo_pre
    global time_point_list

    velo_now = calculate_velo(msg.velo_east, msg.velo_north)
    stamp = msg.header.stamp.secs

    if start_timestamp is None: # first time
        start_timestamp = stamp
        print(start_timestamp)
        if velo_now < speed_thr:
            is_stationary = True
            velo_pre = [0]*(contiguous_length - 1)
        else:
            time_point_list.append(stamp)
        return

    if is_mobile_before() and velo_now > mobile_speed_thr: # mobile
        if is_stationary:
            time_list_append(stamp)
            is_stationary = False
        # else:
        #     print('Not stationary')
    elif is_stationary_before() and velo_now < speed_thr: # stationary
        if not is_stationary:
            time_list_append(stamp)
            is_stationary = True
        # else:
        #     print('Not move')
    updata_velo_pre(velo_now)


def main():
    rospy.init_node('stationary_detector')
    # Define your image topic
    # Set up your subscriber and define its callback
    rospy.Subscriber(pose_topic, LocalizedPose, pose_callback)
    # Spin until ctrl + c
    rospy.spin()


if __name__ == '__main__':
    main()
