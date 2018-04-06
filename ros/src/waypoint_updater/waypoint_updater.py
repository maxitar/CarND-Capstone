#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math

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

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints = None
        self.traffic_wp = -1

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
        angle = lambda a, b: math.atan2(a.y-b.y, a.x-b.x)
        if self.waypoints:
            min_dist = 1e6
            min_idx = -1
            for idx, wp in enumerate(self.waypoints.waypoints):
                dist = dl(msg.pose.position, wp.pose.pose.position)
                if dist  < min_dist:
                    min_dist = dist
                    min_idx = idx
            wp_pose = self.waypoints.waypoints[min_idx].pose.pose.position
            car_pose = msg.pose.position
            wp_to_car_orient = angle(wp_pose, car_pose)
            car_orient = math.acos(msg.pose.orientation.w)*2
            if abs(wp_to_car_orient - car_orient) > math.pi*0.25:
                min_idx += 1
            final_waypoints = Lane()
            final_waypoints.header = self.waypoints.header
            final_waypoints.waypoints = self.waypoints.waypoints[min_idx:min_idx+LOOKAHEAD_WPS]
            #wp_speeds = [wp.twist.twist.linear for wp in final_waypoints.waypoints]
            if self.traffic_wp != -1:
                tl_local_wp = self.traffic_wp-min_idx
                end_wp = max(tl_local_wp - 20, 0)
                # rospy.logwarn("{} {}".format(tl_local_wp, self.traffic_wp))
                vel_step = \
                    final_waypoints.waypoints[tl_local_wp+1].twist.twist.linear.x*0.05
                for idx, wp in enumerate(final_waypoints.waypoints[tl_local_wp:end_wp:-1]):
                    wp.twist.twist.linear.x = idx*vel_step
            else:
                for wp in final_waypoints.waypoints:
                    wp.twist.twist.linear.x = 11
            self.final_waypoints_pub.publish(final_waypoints)
            #for wp, speed in zip(final_waypoints.waypoints, wp_speeds):
            #    wp.twist.twist.linear = speed

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

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
