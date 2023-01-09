#!/usr/bin/env python3
# Not used in the agm-ms3 branch

import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_ros

rospy.init_node('laserscan_to_pointcloud2')
pc2_pub = rospy.Publisher('laser_pc2', PointCloud2, queue_size=1)
laser_proj = laser_geometry.LaserProjection()
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)


def scan_cb(msg):
    try:
        # convert the message of type LaserScan to a PointCloud2
        pc2_msg = laser_proj.projectLaser(msg)
        # transform and publish it
        transform = tf_buffer.lookup_transform('map', pc2_msg.header.frame_id, pc2_msg.header.stamp, rospy.Duration(0.5))
        pc2_msg = do_transform_cloud(pc2_msg, transform)
        pc2_pub.publish(pc2_msg)
    except (tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
        rospy.logwarn(f'Problems with lidar data transformation: {e}')
    except rospy.exceptions.ROSException:
        pass


rospy.Subscriber('laser_ls', LaserScan, scan_cb, queue_size=1)
rospy.spin()
