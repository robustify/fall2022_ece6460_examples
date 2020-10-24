#! /usr/bin/env python

import rospy
import math
import geodesy.utm
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path


class RtkGpsPath:
    def __init__(self):
        rospy.init_node('rtk_gps_path')

        self.ref_lat = rospy.get_param('~ref_lat', default=0.0)
        self.ref_lon = rospy.get_param('~ref_lon', default=0.0)
        self.map_center_utm = geodesy.utm.fromLatLong(self.ref_lat, self.ref_lon)

        self.sub_odom = rospy.Subscriber('/gps/utm_odom', Odometry, self.recv_odom)
        self.pub_path = rospy.Publisher('/ground_truth_path', Path, queue_size=1)
        self.path_msg = Path()

    def recv_odom(self, msg):
        if msg.header.stamp < self.path_msg.header.stamp:
            del self.path_msg.poses[:]
        
        current_x = msg.pose.pose.position.x - self.map_center_utm.easting
        current_y = msg.pose.pose.position.y - self.map_center_utm.northing

        new_pose = PoseStamped()
        new_pose.header.frame_id = 'map'
        new_pose.pose.position.x = current_x
        new_pose.pose.position.y = current_y

        if len(self.path_msg.poses) > 0:
            dx = current_x - self.path_msg.poses[-1].pose.position.x
            dy = current_y - self.path_msg.poses[-1].pose.position.y

            if (dx * dx + dy * dy) > (0.5 * 0.5):
                self.path_msg.poses.append(new_pose)
        else:
            self.path_msg.poses.append(new_pose)

        self.path_msg.header.stamp = msg.header.stamp
        self.path_msg.header.frame_id = 'map'
        self.pub_path.publish(self.path_msg)


if __name__ == '__main__':
    node_instance = RtkGpsPath()
    rospy.spin()
