from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import cv2
import time

import os # For debugging

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

def to_image_coords(boxes, patch_width, patch_height,
                    box_offsets_top, box_offsets_left):
    real_boxes = np.copy(boxes)
    num_patches = boxes.shape[0]
    real_boxes[...,[0,2]] *= patch_height
    real_boxes[...,[1,3]] *= patch_width
    real_boxes[...,[0,2]] += box_offsets_top.reshape((num_patches, 1, 1))
    real_boxes[...,[1,3]] += box_offsets_left.reshape((num_patches, 1, 1))
    return real_boxes

def get_largest_box(boxes):
    diff_x = boxes[:,3]-boxes[:,1]
    diff_y = boxes[:,2]-boxes[:,0]
    box_areas = diff_x*diff_y
    return np.argmax(box_areas)

frame_nr = 0
colors = ((0,0,255),(0,255,255),(0,255,0))
out_folder = os.path.expanduser('~/project/outimg/')+str(time.time())+'/'
os.makedirs(out_folder)
def save_image_with_text(image, boxes, class_nn, class_cv):
    global frame_nr
    for box in boxes:
        top, left, bottom, right = box
        color = colors[class_nn]
        cv2.rectangle(image,(left,top),(right,bottom),color,2)
    largest_box = boxes[get_largest_box(boxes)]
    top, left, bottom, right = largest_box
    color = colors[class_cv]
    cv2.rectangle(image,(left,top),(right,bottom),color,1)

    out_path = out_folder+'{:04}'.format(frame_nr)+'.jpg'
    frame_nr += 1
    cv2.imwrite(out_path, image)

def get_light_color(image, box):
    box = box.astype(int)
    patch = image[box[0]:box[2], box[1]:box[3]]
    lamp_height = patch.shape[0]//3
    r_lamp = patch[lamp_height*2:]
    y_lamp = patch[lamp_height:lamp_height*2]
    g_lamp = patch[0:lamp_height]
    color_threshold = 230
    intesity = np.empty((3))
    # Compute number of bright pixels for each lamp color
    intesity[0] = np.sum(r_lamp>color_threshold)
    intesity[1] = np.sum(y_lamp>color_threshold)
    intesity[2] = np.sum(g_lamp>color_threshold)
    return np.argmax(intesity)+1

class TLClassifier(object):
    def __init__(self, use_nn):
        #TODO load classifier
        self.use_nn = use_nn
        if use_nn:
            nn_path = './light_classification/nn_model/frozen_inference_graph.pb'
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
            config.gpu_options.allow_growth = True
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

    def get_classification_nn(self, image, seq):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Prepare patches
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        patches = []
        left_starts = np.array([0, 300, 0, 300])
        top_starts  = np.array([0, 0, 100, 100])
        patch_height = 500
        patch_width  = 500
        for left, top in zip(left_starts, top_starts):
            patches.append(image[top:top+patch_height, left:left+patch_width])
        # Actual detection.
        start = time.time()
        (boxes, scores, classes) = self.sess.run([self.detection_boxes,
                                                  self.detection_scores,
                                                  self.detection_classes],
                                            feed_dict={self.image_tensor: patches})
        # If no boxes found
        if len(boxes) == 0:
            return TrafficLight.UNKNOWN
        # Change boxes from all patches to image coordinates
        boxes = to_image_coords(boxes, patch_width, patch_height,
                                top_starts, left_starts)
        confidence_cutoff = 0.5
        # Filter boxes with a confidence score less than `confidence_cutoff`
        # or boxes that are either too wide or too thin
        boxes, scores, classes = filter_boxes(confidence_cutoff, boxes, scores, classes)
        # If no high confidence boxes are left
        if len(boxes) == 0:
            return TrafficLight.UNKNOWN
        largest_box_idx = get_largest_box(boxes)
        largest_box = boxes[largest_box_idx]
#        print(largest_box_idx, largest_box)
        state = get_light_color(image, largest_box)
        state_nn = int(round(classes[np.argmax(scores)])+0.001)
#        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
#        state_cv = self.get_classification_cv(image)
#        global frame_nr
#        frame_nr = seq
#        save_image_with_text(image, boxes, 3-state_nn, 3-state)
#        print("CV: {}\tNN: {}\t NN+CV: {}".format(state_cv, 3-state_nn, 3-state))
#        if state == state_nn:
#            print("HIGH CONFIDENCE")
#        print("Detection time: {} Highest confidence: {}"\
#              .format(time.time()-start, np.max(scores)))
        #print(state)
        if state == 3:
            return TrafficLight.RED
        if state == 2:
            return TrafficLight.YELLOW
        if state == 1:
            return TrafficLight.GREEN
        return TrafficLight.UNKNOWN
