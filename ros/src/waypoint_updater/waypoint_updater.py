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
        self.vel = msg.twist.linear.x

    # For debuging
    def brake_cb(self, msg):
        self.brake = msg.pedal_cmd

    def loop(self):
        rate = rospy.Rate(50)
        decel_limit = abs(rospy.get_param('/dbz_node/decel_limit', -5))
        # For debuging
        if DEBUG:
            self.vel = None
            start_time = rospy.get_time()
            out_path = '/home/maxitar/project/sim_data.txt'
            rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
            rospy.Subscriber('/vehicle/brake_cmd', BrakeCmd, self.brake_cb)
            with open(out_path, 'w+') as vel_file:
                vel_file.write('Elapsed DistLight CarVel WpVel DesiredVel Brake')
        while not rospy.is_shutdown():
            if self.waypoints_tree and self.pose:
                min_idx = self.get_closest_waypoint()
                final_waypoints = Lane()
                base_wp = self.waypoints.waypoints[min_idx:min_idx+LOOKAHEAD_WPS]
                if self.traffic_wp != -1 and self.traffic_wp - min_idx <= LOOKAHEAD_WPS:
                    temp = []
                    stop_idx = max(self.traffic_wp - min_idx - 4, 0)
                    for idx, wp in enumerate(base_wp):
                        p = Waypoint()
                        p.pose = wp.pose

                        dist = self.distance(base_wp, idx, stop_idx)
                        vel = math.sqrt(2*decel_limit*dist)
                        if vel < 1.:
                            vel = 0.
                        p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
                        temp.append(p)

                    final_waypoints.waypoints = temp

                    if DEBUG:
                        time_elapsed = rospy.get_time() - start_time
                        next_velocity = final_waypoints.waypoints[0].twist.twist.linear.x
                        car_x = self.pose.pose.position.x
                        car_y = self.pose.pose.position.y
                        stop_wp = base_wp[stop_idx]
                        stop_x = stop_wp.pose.pose.position.x
                        stop_y = stop_wp.pose.pose.position.y
                        dist = math.sqrt((car_x-stop_x)**2 + (car_y-stop_y)**2)
                        #dist = self.distance(base_wp, 0, stop_idx)
                        des_velocity = math.sqrt(2*decel_limit*dist)
                        with open(out_path, 'a+') as vel_file:
                            vel_file.write("{} {} {} {} {} {}\n".format(time_elapsed,
                                                                        dist,
                                                                        self.vel,
                                                                        next_velocity,
                                                                        des_velocity,
                                                                        self.brake))
                else:
                    final_waypoints.waypoints = base_wp

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
        if abs(wp_to_car_orient - car_orient) > math.pi*0.25:
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
