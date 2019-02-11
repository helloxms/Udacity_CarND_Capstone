import rospy

import os
import sys

from styx_msgs.msg import TrafficLight
from PIL import Image
import numpy as np
import tensorflow as tf

from collections import defaultdict
from utils import label_map_util
import cv2
import time
class TLClassifier(object):

    def __init__(self):
        #TODO load classifier
	#print(os.path.abspath('.'))
	cur_path = os.getcwd()
	#print(cur_path)
	#self.PATH_TO_GRAPH = os.path.join(cur_path, r'light_classification/models/ssd_udacity/frozen_inference_graph.pb')
	self.PATH_TO_GRAPH = os.path.join(cur_path, r'light_classification/models/ssd_sim/frozen_inference_graph.pb')
	self.PATH_TO_LABELS = os.path.join(cur_path,  r'light_classification/data/udacity_label_map.pbtxt')
	self.NUM_CLASSES = 13
	self.detection_graph = self.load_graph(self.PATH_TO_GRAPH)

	self.label_map = label_map_util.load_labelmap(self.PATH_TO_LABELS)
	self.categories = label_map_util.convert_label_map_to_categories(self.label_map, max_num_classes=self.NUM_CLASSES, use_display_name=True)
	self.category_index = label_map_util.create_category_index(self.categories)
	self.btest = False

	#test real condition
	#PATH_TO_IMGS =os.path.join(cur_path, r'light_classification/data/udacity_testarea_rgb')
	#PATH_TO_IMGS =os.path.join(cur_path, r'light_classification/data/simulator_dataset_rgb/Red')
	#TEST_IMG = os.path.join(PATH_TO_IMGS, r'left0240.jpg')
	#image = Image.open(TEST_IMG)
	#iLightStat = self.get_classification_test(image)
	#print( self.get_stat_string(iLightStat) )

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
	with self.detection_graph.as_default():
		with tf.Session(graph=self.detection_graph) as sess:
			image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
			detect_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
			detect_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
			detect_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
			num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
					
			#from bgr to rgb image		
			b,g,r = cv2.split(image)
			rgb_img = cv2.merge([r,g,b])
			#image_np = self.load_image_into_numpy_array(image)
			#image_expanded = np.expand_dims(image_np, axis=0)
			image_expanded = np.expand_dims(rgb_img, axis=0)

			(boxes, scores, classes, num) = sess.run(
				[detect_boxes, detect_scores, detect_classes, num_detections],
				feed_dict = {image_tensor: image_expanded})
		

			iLightStat = self.get_light_stat(scores[0][0], int(classes[0][0]))
			stringstate = self.get_stat_string(iLightStat)
			#print( stringstate )
			#rospy.loginfo("get_classification iLightState=%d ", iLightStat )
			#rospy.loginfo( stringstate )
			#cur_path = os.getcwd()
			#sp = time.time()
			#strtime = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(sp))
			#strtime = strtime + stringstate + r'.jpg'
			#cv2.imwrite(os.path.join(cur_path,strtime), image)

			return iLightStat

        return TrafficLight.UNKNOWN


	#IPL image rgb format
	#load image from test file for test train etc
    def get_classification_test(self, image):

	with self.detection_graph.as_default():
		with tf.Session(graph=self.detection_graph) as sess:
			image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
			detect_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
			detect_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
			detect_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
			num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')	

			image_np = self.load_image_into_numpy_array(image)
			image_expanded = np.expand_dims(image_np, axis=0)
			#image_expanded = np.expand_dims(image, axis=0)

			(boxes, scores, classes, num) = sess.run(
				[detect_boxes, detect_scores, detect_classes, num_detections],
				feed_dict = {image_tensor: image_expanded})
		
			print('SCORES')
			print(scores[0])
			print('CLASSES')
			print(classes[0])
			iLightStat = self.get_light_stat(scores[0][0], int(classes[0][0]))
			stringstate = self.get_stat_string(iLightStat)
			print( stringstate )
			rospy.loginfo("get_classification_test iLightState=%d ", iLightStat )
			rospy.loginfo( stringstate )
			return iLightStat

        return TrafficLight.UNKNOWN


    def load_graph(self,graph_file):
	graph = tf.Graph()
	with graph.as_default():
		od_graph_def = tf.GraphDef()
		with tf.gfile.GFile(graph_file, 'rb') as fid:
			serialized_graph = fid.read()
			od_graph_def.ParseFromString(serialized_graph)
			tf.import_graph_def(od_graph_def, name='')
	return graph

	#load image into numpy array
    def load_image_into_numpy_array(self,image):
	(im_width, im_height) = image.size
	return np.array(image.getdata()).reshape((im_height, im_width, 3)).astype(np.uint8)

   
    def get_light_stat(self,score, lightValue):
	tl_class_idx={
		1 : TrafficLight.GREEN,
		2 : TrafficLight.RED,
		3 : TrafficLight.YELLOW,
		4 : TrafficLight.UNKNOWN
	}
	tl_class = TrafficLight.UNKNOWN
	if score >= 0.4:
		tl_class = tl_class_idx.get( lightValue, TrafficLight.UNKNOWN)
	return tl_class

    def get_stat_string(self, stat):
	if stat == 0:
		return "RED"
	elif stat == 1:
		return "YELLOW"
	elif stat == 2:
		return "GREEN"
	else:
		return "UNKNOWN"



if __name__ == '__main__':
        TLClassifier()






