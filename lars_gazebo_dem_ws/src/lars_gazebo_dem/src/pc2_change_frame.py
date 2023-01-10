#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_ros

rospy.init_node('pc2_change_frame')
pc2_pub = rospy.Publisher('laser_pc2', PointCloud2, queue_size=1)
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)


def scan_cb(msg):
    try:
        # transform and publish it
        transform = tf_buffer.lookup_transform('map', msg.header.frame_id, msg.header.stamp, rospy.Duration(0.5))
        msg = do_transform_cloud(msg, transform)
        pc2_pub.publish(msg)
    #except (tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
    #    rospy.logwarn(f'Problems with lidar data transformation: {e}')
    except:
        pass


rospy.Subscriber('laser_pc2_raw', PointCloud2, scan_cb, queue_size=1)
rospy.spin()
