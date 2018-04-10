#!/usr/bin/env python
import math

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
from heapq import heappush, heappop
from scipy.spatial import KDTree

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.waypoints_tree = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        wp2d = [[wp.pose.pose.position.x, wp.pose.pose.position.y] for wp in
                waypoints.waypoints]
        self.waypoints_tree = KDTree(wp2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        return self.waypoints_tree.query([x, y])[1]
#        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
#        min_idx = -1
#        if self.waypoints:
#            min_dist = 1e6
#            for idx, wp in enumerate(self.waypoints.waypoints):
#                dist = dl(pose.position, wp.pose.pose.position)
#                if dist < min_dist:
#                    min_dist = dist
#                    min_idx = idx
#        else:
#            rospy.logwarn("NO WAYPOINTS")
#        return min_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def get_closest_light(self, car_wp):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
        lights = []
        for idx, light in enumerate(self.lights):
            dist = dl(self.pose.pose.position, light.pose.pose.position)
            if dist < 75.:
                heappush(lights,(dist, light, idx))
        while lights:
            light_tuple = heappop(lights)
            light = light_tuple[1]
            light_idx = light_tuple[2]
            # rospy.logwarn(car_wp)
            for idx, wp in enumerate(self.waypoints.waypoints[car_wp:car_wp+150]):
                if dl(wp.pose.pose.position, light.pose.pose.position) <= 20:
                    #return light_idx, light
                    return car_wp+idx, light
#                    wp_orient = math.acos(wp.pose.pose.orientation.w)*2
#                    tl_orient = math.acos(light.pose.pose.orientation.w)*2
#                    angle_diff = wp_orient-tl_orient
#                    if abs(angle_diff) < math.pi/8 or \
#                        abs(math.pi*2-angle_diff) < math.pi/8:
#                        return car_wp+idx, light
        return -1, None

    def get_closest_light2(self, car_wp, stop_line_positions):
        min_wp_dist = len(self.waypoints.waypoints)
        min_idx = -1
        for idx, sl in enumerate(stop_line_positions):
            sl_wp = self.waypoints_tree.query([sl[0], sl[1]])[1]
            if sl_wp - car_wp >= 0 and sl_wp - car_wp < min_wp_dist:
                min_wp_dist = sl_wp - car_wp
                min_idx = idx
        return min_wp_dist + car_wp, self.lights[min_idx]
        dl = lambda a, b: math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
        lights = []
        for idx, stop_line in enumerate(stop_line_positions):
            sl_pos = [stop_line[0], stop_line[1]]
            car_pos = [self.pose.pose.position.x, self.pose.pose.position.y]
            dist = dl(car_pos, sl_pos)
            if dist < 75.:
                heappush(lights,(dist, idx))
        while lights:
            sl_tuple = heappop(lights)
            sl_idx = light_tuple[1]
            for idx, wp in enumerate(self.waypoints.waypoints[car_wp:car_wp+150]):
                wp_pos = [wp.pose.pose.position.x, wp.pose.pose.position.y]
                if dl(wp.pose.pose.position, light.pose.pose.position) <= 20:
                    return light_idx, light
        return -1, None

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if self.pose:
            car_x = self.pose.pose.position.x
            car_y = self.pose.pose.position.y
            car_position = self.get_closest_waypoint(car_x, car_y)
            light_idx, light = self.get_closest_light(car_position)
            light_idx, light = self.get_closest_light2(car_position,
                                                       stop_line_positions)
            # rospy.logwarn("{}, {}".format(car_position, light_idx))
            if light_idx - car_position > 150:
                light = None

        #TODO find the closest visible traffic light (if one exists)

        if light:
            state = self.get_light_state(light)
            return light_idx, state
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
