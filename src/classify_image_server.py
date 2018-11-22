#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import division
from rosie_object_detector.srv import *
import rospy
from std_msgs.msg import Float32
#from __future__ import print_function
import roslib
import numpy
import cv2
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
#from os import path
from pathlib import Path
import argparse
import sys
import time
import numpy as np
import tensorflow as tf

def load_graph():
  model_file="retrained_graph_6_NR.pb"
  #file_path = path.relpath("retrained_graph.pb")
  #print(file_path)
  graph = tf.Graph()
  graph_def = tf.GraphDef()
  #model_file = Path("/home/ras/catkin_ws/src/rosie_object_detector/src/data/retrained_graph_final.pb")
  model_file = Path("/home/ras15/catkin_ws/src/rosie/rosie_object_detector/src/data/retrained_graph_10_NR.pb")
  with open(str(model_file), "rb") as f:
    graph_def.ParseFromString(f.read())
  with graph.as_default():
    tf.import_graph_def(graph_def)
  f.close()
  return graph

def read_tensor_from_image_file(file_name, input_height=299, input_width=299,
				input_mean=0, input_std=255):
  input_name = "file_reader"
  output_name = "normalized"
  file_reader = tf.read_file(file_name, input_name)
  if file_name.endswith(".png"):
    image_reader = tf.image.decode_png(file_reader, channels = 3,
                                       name='png_reader')
  elif file_name.endswith(".gif"):
    image_reader = tf.squeeze(tf.image.decode_gif(file_reader,
                                                  name='gif_reader'))
  elif file_name.endswith(".bmp"):
    image_reader = tf.image.decode_bmp(file_reader, name='bmp_reader')
  else:
    image_reader = tf.image.decode_jpeg(file_reader, channels = 3,
                                        name='jpeg_reader')
  float_caster = tf.cast(image_reader, tf.float32)
  dims_expander = tf.expand_dims(float_caster, 0);
  resized = tf.image.resize_bilinear(dims_expander, [input_height, input_width])
  normalized = tf.divide(tf.subtract(resized, [input_mean]), [input_std])
  sess = tf.Session()
  result = sess.run(normalized)

  return result

def load_labels():
  #label_file = "~/catkin_ws/src/rosie_object_classifier/src/retrained_labels.txt"
  #file_path = path.relpath("retrained_graph.pb")
  label = []
  #model_file = Path("/home/ras/catkin_ws/src/rosie_object_detector/src/data/retrained_labels.txt")
  model_file = Path("/home/ras15/catkin_ws/src/rosie/rosie_object_detector/src/data/retrained_labels.txt")
  proto_as_ascii_lines = tf.gfile.GFile(str(model_file)).readlines()
  for l in proto_as_ascii_lines:
    label.append(l.rstrip())
  return label

#if __name__ == "__main__":
def overall_call(file_name):
  print('Entered here')
  #file_name = "tf_files/flower_photos/daisy/3475870145_685a19116d.jpg"
  #model_file = "tf_files/retrained_graph.pb"
  #label_file = "tf_files/retrained_labels.txt"
  input_height = 224
  input_width = 224
  input_mean = 128
  input_std = 128
  input_layer = "input"
  output_layer = "final_result"

  #parser = argparse.ArgumentParser()
  #parser.add_argument("--image", help="image to be processed")
  #parser.add_argument("--graph", help="graph/model to be executed")
  #parser.add_argument("--labels", help="name of file containing labels")
  #parser.add_argument("--input_height", type=int, help="input height")
  #parser.add_argument("--input_width", type=int, help="input width")
  #parser.add_argument("--input_mean", type=int, help="input mean")
  #parser.add_argument("--input_std", type=int, help="input std")
  #parser.add_argument("--input_layer", help="name of input layer")
  #parser.add_argument("--output_layer", help="name of output layer")
  #args = parser.parse_args()

  #if args.graph:
  #  model_file = args.graph
  #if args.image:
  #  file_name = args.image
  #if args.labels:
  #  label_file = args.labels
  #if args.input_height:
  #  input_height = args.input_height
  #if args.input_width:
  #  input_width = args.input_width
  #if args.input_mean:
  #  input_mean = args.input_mean
  #if args.input_std:
  #  input_std = args.input_std
  #if args.input_layer:
  #  input_layer = args.input_layer
  #if args.output_layer:
  #  output_layer = args.output_layer
  #print(model_file)

  graph = load_graph()
  t = read_tensor_from_image_file(file_name,
                                  input_height=input_height,
                                  input_width=input_width,
                                  input_mean=input_mean,
                                  input_std=input_std)

  input_name = "import/" + input_layer
  output_name = "import/" + output_layer
  input_operation = graph.get_operation_by_name(input_name);
  output_operation = graph.get_operation_by_name(output_name);
  print('Successful here')
  with tf.Session(graph=graph) as sess:
    start = time.time()
    results = sess.run(output_operation.outputs[0],
                      {input_operation.outputs[0]: t})
    end=time.time()
  results = np.squeeze(results)

  top_k = results.argsort()[-5:][::-1]
  shapes_k = []
  colors_k = []
  top_results = []
  #sort_list = results.argsort()
  #sorted_list = results[sort_list]
  labels = load_labels()

  print('\nEvaluation time (1-image): {:.3f}s\n'.format(end-start))
  template = "{} (score={:0.5f})"
  for i in top_k:
    print(template.format(labels[i], results[i]))
    top_results.append(labels[i])
  #print("top results are")
  #print(top_results)	

  for i in top_results:
	#print (i)
	if(i == 'orangestar'):
		shapes_k.append("Star")
		colors_k.append(1)
	if(i == 'purpleplus'):
		shapes_k.append("Plus")
		colors_k.append(6)
	if(i == 'greencube'):
		shapes_k.append("Cube")
		colors_k.append(4)
	if(i == 'greenhollowcube'):
		shapes_k.append("HollowCube")
		colors_k.append(4)
	if(i == 'redhollowcube'):
		shapes_k.append("HollowCube")
		colors_k.append(0)
	if(i == 'redsphere'):
		shapes_k.append("Sphere")
		colors_k.append(0)
	if(i == 'bluetriangle'):
		shapes_k.append("Sphere")
		colors_k.append(5)
	if(i == 'greenhollowcylinder'):
		shapes_k.append("Sphere")
		colors_k.append(3)
	if(i == 'orangeplus'):
		shapes_k.append("Sphere")
		colors_k.append(1)
	if(i == 'yellowsphere'):
		shapes_k.append("Sphere")
		colors_k.append(2)
	if(i == 'battery'):
		shapes_k.append("Battery")
		colors_l.append(7)



  return results,labels,top_k,shapes_k,colors_k, top_results

def decideOnObject(index, colors, shapes, results, labels, top_indices):
	if(index == 0 or index == 1 or index == 2):
		if(colors[0] == index):
			if(results[top_indices[0]] > 0.5):
				print "Perfect Match"
				return labels[top_indices[0]]
			else:
				print "Good Match"
				return labels[top_indices[0]]
	elif(index == 3 or index == 4):
		if(colors[0] == index):
			if(results[top_indices[0]] > 0.5):
				print "Perfect Match"
				return labels[top_indices[0]]
			else:
				print "Good Match"
				return labels[top_indices[0]]
	elif(index == 5 or index == 6):
		if(colors[0] == index):
			if(results[top_indices[0]] > 0.5):
				print "Perfect Match"
				return labels[top_indices[0]]
			else:
				print "Good Match"
				return labels[top_indices[0]]
	elif(index == 7):
		if(colors[0] == index):
			if(results[top_indices[0]] > 0.5):
				print "Perfect Match"
				return labels[top_indices[0]]
			else:
				print "Good Match"
				return labels[top_indices[0]]

	if(index == 0 or index == 1 or index == 2):
		if(colors[1] == index):
			if(results[top_indices[1]] > 0.5):
				print "Good Match"
				return labels[top_indices[1]]
			else:
				print "Bad Match"
				return labels[top_indices[1]]
	elif(index == 3 or index == 4):
		if(colors[1] == index):
			if(results[top_indices[1]] > 0.5):
				print "Good Match"
				return labels[top_indices[1]]
			else:
				print "Bad Match"
				return labels[top_indices[1]]
	elif(index == 5 or index == 6):
		if(colors[1] == index):
			if(results[top_indices[1]] > 0.5):
				print "Good Match"
				return labels[top_indices[1]]
			else:
				print "Bad Match"
				return labels[top_indices[1]]
	elif(index == 7):
		if(colors[1] == index):
			if(results[top_indices[1]] > 0.5):
				print "Good Match"
				return labels[top_indices[1]]
			else:
				print "Bad Match"
				return labels[top_indices[1]]
	if(index == 0 or index == 1 or index == 2):
		if(colors[2] == index):
			if(results[top_indices[2]] > 0.5):
				print "Bad Match"
				return labels[top_indices[2]]
			else:
				print "Very Bad Match"
				return labels[top_indices[2]]
	elif(index == 3 or index == 4):
		if(colors[2] == index):
			if(results[top_indices[2]] > 0.5):
				print "Bad Match"
				return labels[top_indices[2]]
			else:
				print "Very Bad Match"
				return labels[top_indices[2]]
	elif(index == 5 or index == 6):
		if(colors[2] == index):
			if(results[top_indices[2]] > 0.5):
				print "Bad Match"
				return labels[top_indices[2]]
			else:
				print "Very Bad Match"
				return labels[top_indices[2]]
	elif(index == 7):
		if(colors[2] == index):
			if(results[top_indices[2]] > 0.5):
				print "Bad Match"
				return labels[top_indices[2]]
			else:
				print "Very Bad Match"
				return labels[top_indices[2]]
		
		


def handle_classify_image(req):
	print("Received an image")
	bridge = CvBridge()
	request = ObjectClassifyRequest()
	print req
	resp = ObjectClassifyResponse()
	#a, b, top_k, shapes_k, colors_k, top_results = overall_call("/home/ras/catkin_ws/src/rosie_object_detector/CameraCapture/camera_capture_%d.jpg"%req.img_number.data)
	a, b, top_k, shapes_k, colors_k, top_results = overall_call("/home/ras15/catkin_ws/src/rosie/rosie_object_detector/CameraCapture/camera_capture_%d.jpg"%req.img_number.data)
	#OriginalImage = cv2.imread("/home/ras/catkin_ws/src/rosie_object_detector/CameraCapture/camera_capture_%d.jpg"%req.img_number.data)
	#print OriginalImage.shape
	#cv2.imshow("image_to_be_classified", OriginalImage)
	#font = cv2.FONT_HERSHEY_SIMPLEX
	#cv2.putText(OriginalImage,"%d"%req.color_ind.data,(200,200), font, 2,(255,255,255),2,cv2.LINE_AA)
	#cv2.putText(OriginalImage,"%s"%b[top_k[0]],(200,200), font, 2,(255,0,255),2,cv2.LINE_AA)
	#cv2.waitKey(0)
	#print("the top color indices are")
	#print(colors_k)
        #print("The color detected by the detector is")
	#print(req.color_ind.data)
	shape_id = decideOnObject(req.color_ind.data, colors_k, shapes_k, a, b, top_k)
	print("The shape_id identified is")
	print(shape_id)
	resp.perc1.data = a[top_k[0]]
	resp.perc2.data = a[top_k[1]]
	resp.perc3.data = a[top_k[2]]
	resp.first_shape.data = shapes_k[0]
	resp.second_shape.data = shapes_k[1]
	resp.third_shape.data = shapes_k[2]
	resp.first_color.data = colors_k[0]
	resp.second_color.data = colors_k[1]
	resp.third_color.data = colors_k[2]
	x = String()
	x.data = str(shape_id)
	resp.decision = x
	print resp
	#os.remove("/home/ras/catkin_ws/src/rosie_object_detector/CameraCapture/camera_capture_%d.jpg"%req.img_number.data)
	os.remove("/home/ras15/catkin_ws/src/rosie/rosie_object_detector/CameraCapture/camera_capture_%d.jpg"%req.img_number.data)
	return resp

def classify_image_server():
	rospy.init_node('do_something_with_image_server')
	s = rospy.Service('do_something_with_image', ObjectClassify, handle_classify_image)
        print("Ready to do something with an image")
	rospy.spin()
	#cv2.destroyAllWindows()

if __name__ == "__main__":
#    add_two_floats_server()
	classify_image_server()
