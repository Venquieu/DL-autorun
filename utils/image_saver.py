#!/usr/bin/env python
import rospy
# ROS Image message
from sensor_msgs.msg import Image, CompressedImage
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
import cv2
import argparse
import defaults

# Instantiate CvBridge
image_topic = "/aw_cam03/image_raw/compressed"
save_interval = defaults.image_save_interval #second
last_save_time = -1
bag_name = None

bridge = CvBridge()

def image_callback(msg):
    global last_save_time
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.compressed_imgmsg_to_cv2(msg)
    except CvBridgeError as e:
        print(e)
    else:
        time = msg.header.stamp
        if time.to_sec() - last_save_time > save_interval:
            cv2.imwrite(bag_name+'_'+str(time) + '.jpg', cv2_img)
            last_save_time = time.to_sec()

def print_info():
    print('Subscribted topic:', image_topic)

def main():
    rospy.init_node('image_listener')
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, CompressedImage, image_callback)
    # Spin until ctrl + c
    print_info()
    rospy.spin()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Image saver.")
    parser.add_argument(
        'bag_name', help="name of the bag"
    )
    args = parser.parse_args()
    bag_name = args.bag_name
    main()
