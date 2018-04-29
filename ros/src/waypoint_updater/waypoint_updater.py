#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree

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
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.waypoints = None
        self.waypoints_tree = None
        self.pose = None
        self.traffic_wp = -1

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # Publish at 50Hz
        a0 = -3 # m/s^2; constant acceleration coefficient
        while not rospy.is_shutdown():
            if self.waypoints_tree and self.pose:
                closest_wp = self.get_closest_waypoint()
                final_waypoints = Lane()
                base_waypoints = self.waypoints.waypoints[closest_wp:closest_wp+LOOKAHEAD_WPS]
                if self.traffic_wp != -1 and self.traffic_wp - closest_wp <= LOOKAHEAD_WPS:
                    temp = []
                    stop_idx = max(self.traffic_wp - closest_wp - 4, 0)
                    # Precompute distances of all waypoints to traffic light
                    # to avoid O(n^2) computations of each distance individually
                    distances = self.get_relative_distances(base_waypoints,
                                                            stop_idx)
                    for idx, wp in enumerate(base_waypoints):
                        p = Waypoint()
                        p.pose = wp.pose

                        dist = distances[idx]
                        # Velocity for constant deceleration
                        target_velocity = math.sqrt(-2*a0*dist)
                        if target_velocity < 1.:
                            target_velocity = 0.
                        p.twist.twist.linear.x = min(target_velocity, wp.twist.twist.linear.x)
                        temp.append(p)

                    final_waypoints.waypoints = temp

                else:
                    final_waypoints.waypoints = base_waypoints

                self.final_waypoints_pub.publish(final_waypoints)
            rate.sleep()

    def get_closest_waypoint(self):
        car_x = self.pose.pose.position.x
        car_y = self.pose.pose.position.y
        min_idx = self.waypoints_tree.query([car_x, car_y])[1]
        wp_pose1 = self.waypoints.waypoints[min_idx].pose.pose.position
        vec1 = [car_x - wp_pose1.x, car_y - wp_pose1.y]
        wp_pose2 = self.waypoints.waypoints[min_idx+1].pose.pose.position
        vec2 = [wp_pose2.x - wp_pose1.x, wp_pose2.y - wp_pose1.y]
        prod = vec1[0]*vec2[0] + vec1[1]*vec2[1]
        if prod > 0:
            min_idx += 1
        return min_idx

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        wp2d = [[wp.pose.pose.position.x, wp.pose.pose.position.y] for wp in
                waypoints.waypoints]
        self.waypoints_tree = KDTree(wp2d)

    def traffic_cb(self, msg):
        self.traffic_wp = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def get_relative_distances(self, waypoints, anchor_wp):
        '''
        Computes the distances of all nodes in 'waypoints' to anchor_wp in
        linear time

        Args:
            waypoints (Waypoint[]): array of waypoints
            anchor_wp (int): index of anchor waypoint in 'waypoints'

        Returns:
            list: relative Euclidean distances of all waypoints to anchor_wp
        '''
        dist = 0
        distances = [0 for i in range(len(waypoints))]
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(anchor_wp, 0, -1):
            dist += dl(waypoints[i].pose.pose.position, waypoints[i-1].pose.pose.position)
            distances[i-1] = dist
        return distances


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
