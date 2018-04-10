from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import cv2

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        # self.get_network()
        pass

    def get_network(self):
        self.x = tf.placeholder(tf.float32, (None, 600, 800, 3))
        self.y = tf.placeholder(tf.int32, (None))

        conv1 = tf.layers.conv2d(self.x, 8, (3, 3), padding='same')
        conv1 = tf.nn.relu(conv1)
        pool1 = tf.layers.max_pooling2d(conv1, (2, 2), (2, 2))

        conv2 = tf.layers.conv2d(pool1, 8, (3, 3), padding='same')
        conv2 = tf.nn.relu(conv2)
        pool2 = tf.layers.max_pooling2d(conv2, (2, 2), (2, 2))

        conv3 = tf.layers.conv2d(pool2, 8, (3, 3), padding='same')
        conv3 = tf.nn.relu(conv3)
        pool3 = tf.layers.max_pooling2d(conv3, (2, 2), (2, 2))

        conv4 = tf.layers.conv2d(pool3, 8, (3, 3), padding='same')
        conv4 = tf.nn.relu(conv4)
        pool4 = tf.layers.max_pooling2d(conv4, (2, 2), (2, 2))

        conv5 = tf.layers.conv2d(pool4, 8, (3, 3), padding='same')
        conv5 = tf.nn.relu(conv5)
        pool5 = tf.layers.max_pooling2d(conv5, (2, 2), (2, 2))

        shape = pool5.get_shape().as_list()
        dim = np.prod(shape[1:])
        flat5 = tf.reshape(pool5, [-1, dim])
        dense1 = tf.layers.dense(flat5, 10)
        dense1 = tf.nn.relu(dense1)

        self.logits = tf.layers.dense(dense1, 4)
        self.sess = tf.Session()
        saver = tf.train.Saver()
        saver.restore(self.sess, './light_classification/tlnet')

        self.mean = np.load('./light_classification/img_mean.npy')
        self.std = np.load('./light_classification/img_std.npy')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        bool_img = (image[:,:,2] >= 200) & (image[:,:,1] <= 70)
        if np.sum(bool_img) >= 27:
            return TrafficLight.RED
        bool_img = (image[:,:,1] >= 235) & (image[:,:,2] <= 120)
        if np.sum(bool_img) >= 30:
            return TrafficLight.GREEN
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        bool_img = (image[:,:,0] >= 28) & (image[:,:,0] <= 32) & (image[:,:,2] >= 240)
        if np.sum(bool_img) >= 40:
            return TrafficLight.YELLOW
        return TrafficLight.UNKNOWN

    def get_classification_nn(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        image = image.astype(np.float32)
        image -= self.mean
        image /= self.std
        image = image.reshape((1, 600, 800, 3))
        logits = self.sess.run(self.logits, feed_dict={self.x:image})
        state = np.argmax(logits)
        # rospy.logwarn("{}".format(state))
        print state
        print logits
        if state == 3:
            return TrafficLight.RED
        if state == 2:
            return TrafficLight.YELLOW
        if state == 1:
            return TrafficLight.GREEN
        return TrafficLight.UNKNOWN
