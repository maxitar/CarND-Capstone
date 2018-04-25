#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree

import math

DEBUG = False
#DEBUG = True
if DEBUG:
    from geometry_msgs.msg import TwistStamped
    from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd

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

    # For debuging
    def velocity_cb(self, msg):
        self.velocity = msg.twist.linear.x

    # For debuging
    def brake_cb(self, msg):
        self.brake = msg.pedal_cmd

    def loop(self):
        rate = rospy.Rate(50)
        a0 = -3 # m/s^2; constant acceleration coefficient
        # For debuging
        if DEBUG:
            self.velocity = None
            start_time = rospy.get_time()
            out_path = '/home/maxitar/project/sim_data.txt'
            rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
            rospy.Subscriber('/vehicle/brake_cmd', BrakeCmd, self.brake_cb)
            with open(out_path, 'w+') as vel_file:
                vel_file.write('Elapsed DistLight CarVel WpVel DesiredVel Brake')
        while not rospy.is_shutdown():
            if self.waypoints_tree and self.pose:
                min_idx = self.get_closest_waypoint()
                min_idx_vec = self.get_closest_waypoint_vec()
                #rospy.logwarn("{} {}".format(min_idx, min_idx_vec))
                min_idx = min_idx_vec
                final_waypoints = Lane()
                base_waypoints = self.waypoints.waypoints[min_idx:min_idx+LOOKAHEAD_WPS]
                if self.traffic_wp != -1 and self.traffic_wp - min_idx <= LOOKAHEAD_WPS:
                    temp = []
                    stop_idx = max(self.traffic_wp - min_idx - 4, 0)
                    # Precompute distances of all waypoints to traffic light
                    # to avoid O(n^2) computations of each distance individually
                    distances = self.get_relative_distances(base_waypoints,
                                                            stop_idx)
                    for idx, wp in enumerate(base_waypoints):
                        p = Waypoint()
                        p.pose = wp.pose

                        dist = distances[idx]
                        # For debugging
                        #dist = self.distance(base_waypoints, idx, stop_idx)
                        #rospy.logwarn('{} {}'.format(dist, distances[idx]))
                        # Velocity for constant deceleration
                        target_velocity = math.sqrt(-2*a0*dist)
                        if target_velocity < 1.:
                            target_velocity = 0.
                        p.twist.twist.linear.x = min(target_velocity, wp.twist.twist.linear.x)
                        temp.append(p)

                    final_waypoints.waypoints = temp

                    if DEBUG:
                        time_elapsed = rospy.get_time() - start_time
                        next_velocity = final_waypoints.waypoints[0].twist.twist.linear.x
                        car_x = self.pose.pose.position.x
                        car_y = self.pose.pose.position.y
                        stop_wp = base_waypoints[stop_idx]
                        stop_x = stop_wp.pose.pose.position.x
                        stop_y = stop_wp.pose.pose.position.y
                        dist = math.sqrt((car_x-stop_x)**2 + (car_y-stop_y)**2)
                        #dist = self.distance(base_waypoints, 0, stop_idx)
                        des_velocity = math.sqrt(-2*a0*dist)
                        with open(out_path, 'a+') as vel_file:
                            vel_file.write("{} {} {} {} {} {}\n".format(time_elapsed,
                                                                        dist,
                                                                        self.velocity,
                                                                        next_velocity,
                                                                        des_velocity,
                                                                        self.brake))
                else:
                    final_waypoints.waypoints = base_waypoints

                self.final_waypoints_pub.publish(final_waypoints)
            rate.sleep()

    def get_closest_waypoint(self):
        angle = lambda a, b: math.atan2(a.y-b.y, a.x-b.x)
        car_x = self.pose.pose.position.x
        car_y = self.pose.pose.position.y
        min_idx = self.waypoints_tree.query([car_x, car_y])[1]
        wp_pose = self.waypoints.waypoints[min_idx].pose.pose.position
        car_pose = self.pose.pose.position
        wp_to_car_orient = angle(wp_pose, car_pose)
        car_orient = math.acos(self.pose.pose.orientation.w)*2
        # For debugging
#        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
#        rospy.logwarn(dl(wp_pose, car_pose))
        #
        if abs(wp_to_car_orient - car_orient) > math.pi*0.25:
            # For debugging
#            rospy.logwarn("Original closest: {}".format(min_idx))
#            wp1_x = wp_pose.x
#            wp1_y = wp_pose.y
#            wp_pose = self.waypoints.waypoints[min_idx+1].pose.pose.position
#            wp2_x = wp_pose.x
#            wp2_y = wp_pose.y
#            rospy.logwarn("WP min: {},{} WP next: {},{} CAR: {},{}".format(
#                wp1_x, wp1_y, wp2_x, wp2_y, car_x, car_y))
            #
            min_idx += 1
        return min_idx

    def get_closest_waypoint_vec(self):
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

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

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
