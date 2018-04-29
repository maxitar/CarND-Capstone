from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import cv2

def filter_boxes(min_score, boxes, scores, classes):
    """Return boxes with a confidence >= `min_score`"""
    above_threshold = scores > min_score
    filtered_boxes = boxes[above_threshold]
    filtered_scores = scores[above_threshold]
    filtered_classes = classes[above_threshold]
    diff_x = filtered_boxes[:,3]-filtered_boxes[:,1]
    diff_y = filtered_boxes[:,2]-filtered_boxes[:,0]
    ratio = diff_y/diff_x
    # Throw out too thin or too wide boxes, e.g. if only 2/3rds of TL detected
    bad_ratio = (ratio<2.2) | (ratio>3.1)
    filtered_boxes = filtered_boxes[~bad_ratio]
    filtered_scores = filtered_scores[~bad_ratio]
    filtered_classes = filtered_classes[~bad_ratio]
    return filtered_boxes, filtered_scores, filtered_classes

def load_graph(graph_file):
    """Loads a frozen inference graph"""
    graph = tf.Graph()
    with graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(graph_file, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
    return graph

def to_image_coords(boxes, patch_widths, patch_heights,
                    box_offsets_top, box_offsets_left):
    """
    The boxes returned from the classifier are normalized in [0, 1] range
    with respect to the patch they belong to. This function scales them to
    patch size and then shifts them to image coordinates.
    """
    real_boxes = np.copy(boxes)
    num_patches = boxes.shape[0]
    real_boxes[...,[0,2]] *= patch_heights.reshape((num_patches, 1, 1))
    real_boxes[...,[1,3]] *= patch_widths.reshape((num_patches, 1, 1))
    real_boxes[...,[0,2]] += box_offsets_top.reshape((num_patches, 1, 1))
    real_boxes[...,[1,3]] += box_offsets_left.reshape((num_patches, 1, 1))
    return real_boxes

def get_largest_box(boxes):
    """Returns the index of the largest box with respect to area"""
    diff_x = boxes[:,3]-boxes[:,1]
    diff_y = boxes[:,2]-boxes[:,0]
    box_areas = diff_x*diff_y
    return np.argmax(box_areas)

def get_light_color(image, box):
    box = box.astype(int)
    patch = image[box[0]:box[2], box[1]:box[3]]
    light_height = patch.shape[0]
    r_lamp = patch[:int(0.33*light_height)]
    y_lamp = patch[int(0.33*light_height):int(0.66*light_height)]
    g_lamp = patch[int(0.66*light_height):]
    color_threshold = 230
    intesity = np.empty((3))
    # Compute number of bright pixels for each lamp color
    intesity[0] = np.sum(g_lamp>color_threshold)
    intesity[1] = np.sum(y_lamp>color_threshold)
    intesity[2] = np.sum(r_lamp>color_threshold)
    return np.argmax(intesity)+1

class TLClassifier(object):
    def __init__(self, use_nn, tf13 = True):
        self.use_nn = use_nn
        if use_nn:
            self.tf13 = tf13
            if tf13:
                # For use with TensorFlow 1.3+
                nn_path = './light_classification/nn_model/frozen_inference_graph_tf13.pb'
            else:
                # Alternative model for TensorFlow 1.7+
                nn_path = './light_classification/nn_model/frozen_inference_graph_tf17.pb'
            self.graph = load_graph(nn_path)
            # The input placeholder for the image.
            # `get_tensor_by_name` returns the Tensor with the associated name in the Graph.
            self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
            # Each box represents a part of the image where a particular object was detected.
            self.detection_boxes = self.graph.get_tensor_by_name('detection_boxes:0')
            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            self.detection_scores = self.graph.get_tensor_by_name('detection_scores:0')
            # The classification of the object (integer id).
            self.detection_classes = self.graph.get_tensor_by_name('detection_classes:0')
            config = tf.ConfigProto()
            # Dynamic memory allocation, but can fragment the memory
            # config.gpu_options.allow_growth = True
            self.sess = tf.Session(graph=self.graph, config=config)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if self.use_nn:
            return self.get_classification_nn(image)
        else:
            return self.get_classification_cv(image)

    def get_classification_cv(self, image):
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
        # Prepare patches
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        patches = []
        # Uncomment for better precision but slower inference
#        left_starts = np.array([0, 300, 0, 300, 0, 0, 200])
#        top_starts  = np.array([0, 0, 100, 100, 0, 0, 0])
#        patch_heights = np.array([500, 500, 500, 500, 600, 600, 600])
#        patch_widths  = np.array([500, 500, 500, 500, 800, 600, 600])
        left_starts = np.array([0, 300, 0, 300, 0])
        top_starts  = np.array([0, 0, 100, 100, 0])
        patch_heights = np.array([500, 500, 500, 500, 600])
        patch_widths  = np.array([500, 500, 500, 500, 800])
        for left, top, patch_width, patch_height in zip(left_starts, top_starts,
                                                        patch_widths, patch_heights):
            patches.append(cv2.resize(image[top:top+patch_height,
                                            left:left+patch_width],(300, 300)))
        # Actual detection.
        (boxes, scores, classes) = self.sess.run([self.detection_boxes,
                                                  self.detection_scores,
                                                  self.detection_classes],
                                            feed_dict={self.image_tensor: patches})
        # If no boxes found
        if len(boxes) == 0:
            return TrafficLight.UNKNOWN
        # Change boxes from all patches to image coordinates
        boxes = to_image_coords(boxes, patch_widths, patch_heights,
                                top_starts, left_starts)
        if self.tf13:
            confidence_cutoff = 0.3
        else:
            confidence_cutoff = 0.5
        # Filter boxes with a confidence score less than `confidence_cutoff`
        # or boxes that are either too wide or too thin
        boxes, scores, classes = filter_boxes(confidence_cutoff, boxes, scores, classes)
        # If no high confidence boxes are left
        if len(boxes) == 0:
            return TrafficLight.UNKNOWN
        if self.tf13:
            # Direct use of classification works better for TF1.3 model
            state = int(classes[np.argmax(scores)])
        else:
            # Calculating the most bright pixels in each lamp section
            # works better for TF1.7 model for the provided rosbag
            largest_box_idx = get_largest_box(boxes)
            largest_box = boxes[largest_box_idx]
            state = get_light_color(image, largest_box)
        if state == 3:
            return TrafficLight.RED
        if state == 2:
            return TrafficLight.YELLOW
        if state == 1:
            return TrafficLight.GREEN
        return TrafficLight.UNKNOWN
