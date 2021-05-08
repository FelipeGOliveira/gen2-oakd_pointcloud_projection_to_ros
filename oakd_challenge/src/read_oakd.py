#!/usr/bin/env python3
# Author: Felipe Gomes de Oliveira
#==================================== INPUTS ===============================================
# Text file with Intrinsic Parameters
# /home/catkin_ws/src/oakd_calibration/left/
# /home/catkin_ws/src/oakd_calibration/right/
#==================================== OUTPUTS ==============================================
# PointCloud2 topic
# /pointcloud
#==================================== GOAL =================================================
# Publish point cloud from OAK-D camera.
#===========================================================================================

import numpy as np
import csv
import os 
import sys

import cv2
import depthai as dai

import rospy
from sensor_msgs.msg import PointCloud2, PointField

dirname = os.path.dirname(__file__) 
pathLibs = (dirname + '/libs/')
sys.path.append(pathLibs)

import pointcloud_msg_lib as pc
import camerainfo_lib as cam_info
import oakd_lib as oakd


class OAKD_Reader():

	def __init__(self):

		# Start defining a pipeline
		self.pipeline = dai.Pipeline()

		self.camLeft = oakd.left_Camera_Defitions(self.pipeline)
		self.camRight = oakd.right_Camera_Defitions(self.pipeline)

		self.pointcloud_pub = rospy.Publisher('point_cloud', PointCloud2, queue_size=10)

	
	def images_Capture(self, pipeline, right_cam_info, camera_matrix2, camera_matrixMat):
	
		oakd.depth_definition(pipeline, self.camLeft, self.camRight)
		
##		oakd.right_cam_definition(pipeline, self.camRight)

		print("---------------------------------------------------------------------------")
		print("...Depth Map to Point Cloud...Publishing /pointcloud topic...")

		# Pipeline defined, now the device is connected to
		with dai.Device(pipeline) as device:

			# Start pipeline
			device.startPipeline()

##			qRight = device.getOutputQueue(name="right", maxSize=4, blocking=False)
			q = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

			invK = np.linalg.inv( camera_matrixMat )

			while True:
				inDepth = q.get()
##				inRight = qRight.get()

				frame2d = inDepth.getFrame()
##				frameRight = inRight.getFrame()

				points = pc.createPointCloudData(frame2d, invK)				
				
				stamp = rospy.Time.now()
				pc.publishPointCloud(stamp, points, self.pointcloud_pub)


##				cv2.imshow("Depth Map	", frame2d)
##				if cv2.waitKey(1) == ord('q'):
##					break


		
			

def main():

	dirname = os.path.dirname(__file__) 
	pathLeft = (dirname + '/oakd_calibration/left/camera_info.txt')
	pathRight = (dirname + '/oakd_calibration/right/camera_info.txt')

	rospy.init_node('reading_oakd_node')
	
	obj = OAKD_Reader()
	
	# ________________________ LOAD CAMERA PARAMETERS ________________________
	right_cam_info, camera_matrix2, camera_matrixMat = cam_info.loadCameraInfo(pathRight)

	# ________________________ IMAGE AND DEPTH CAPTURE ________________________
	obj.images_Capture(obj.pipeline, right_cam_info, camera_matrix2, camera_matrixMat)

	rospy.spin()
	

if __name__ == '__main__':
	main()
	

