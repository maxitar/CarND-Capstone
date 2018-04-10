#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree

import math
from copy import deepcopy

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.waypoints = None
        self.waypoints_tree = None
        self.traffic_wp = -1

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
        angle = lambda a, b: math.atan2(a.y-b.y, a.x-b.x)
        if self.waypoints:
            car_x = msg.pose.position.x
            car_y = msg.pose.position.y
            min_idx = self.waypoints_tree.query([car_x, car_y])[1]
            wp_pose = self.waypoints.waypoints[min_idx].pose.pose.position
            car_pose = msg.pose.position
            wp_to_car_orient = angle(wp_pose, car_pose)
            car_orient = math.acos(msg.pose.orientation.w)*2
            if abs(wp_to_car_orient - car_orient) > math.pi*0.25:
                min_idx += 1
            final_waypoints = Lane()
            # final_waypoints.header = self.waypoints.header
            tl_local_wp = self.traffic_wp - min_idx# - 3
            base_wp = self.waypoints.waypoints[min_idx:min_idx+LOOKAHEAD_WPS]
            if tl_local_wp >= 0:
                # final_waypoints.header = copy.deepcopy(final_waypoints.header)
#                 temp = []
#                 for idx, wp in enumerate(base_wp):
#                     p = Waypoint()
#                     p.pose = wp.pose
# 
#                     stop_idx = max(self.traffic_wp - min_idx - 2, 0)
#                     dist = self.distance(base_wp, idx, stop_idx)
#                     vel = math.sqrt(2*5*dist)
#                     if vel < 1. or stop_idx <= idx:
#                         vel = 0.
#                     p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
#                     temp.append(p)
#                 final_waypoints.waypoints = temp
                 final_waypoints.waypoints = \
                     deepcopy(self.waypoints.waypoints[min_idx:min_idx+LOOKAHEAD_WPS])
                 end_wp = max(tl_local_wp - 50, 0)
                 # final_waypoints.waypoints = copy.deepcopy(final_waypoints.waypoints)
                 # final_waypoints = copy.deepcopy(final_waypoints)
                 rospy.logwarn("{} {}".format(tl_local_wp, self.traffic_wp))
                 vel_step = \
                     final_waypoints.waypoints[tl_local_wp+1].twist.twist.linear.x*0.02
                 tl_local_wp = max(tl_local_wp - 3, 0)
                 for idx, wp in enumerate(final_waypoints.waypoints[tl_local_wp:end_wp:-1]):
                     wp.twist.twist.linear.x = int(idx*vel_step)
                 for wp in final_waypoints.waypoints[tl_local_wp+1:]:
                     wp.twist.twist.linear.x = 0.
                 for idx, wp in enumerate(final_waypoints.waypoints):
                     wp.pose = base_wp[idx].pose
            else:
                # final_waypoints.waypoints = self.waypoints.waypoints[min_idx:min_idx+LOOKAHEAD_WPS]
                # final_waypoints.waypoints = deepcopy(base_wp)
                temp = []
                for wp in base_wp:
                    p = Waypoint()
                    p.pose = wp.pose
                    p.twist.twist.linear.x = wp.twist.twist.linear.x
                    temp.append(p)
                final_waypoints.waypoints = temp

            self.final_waypoints_pub.publish(final_waypoints)

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        wp2d = [[wp.pose.pose.position.x, wp.pose.pose.position.y] for wp in
                waypoints.waypoints]
        self.waypoints_tree = KDTree(wp2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_wp = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
