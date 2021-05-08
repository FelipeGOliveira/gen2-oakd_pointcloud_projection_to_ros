#!/usr/bin/env python3
import numpy as np
import csv
import rospy
from sensor_msgs.msg import CameraInfo



def loadCameraInfo(path):
	print("---------------------------------------------------------------------------")
	print("...Loading Intrinsic Parameters...")
	camera_matrix = []
	camera_matrix2 = []
	distortion = []
	with open(path) as csvfile:
		spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
		for i,row in enumerate(spamreader):

			if(i == 2):
				height = int(row[0])
				width = int(row[1])
			if(i == 3):
				camera_matrix = [float(row[0]), float(row[1]), float(row[2]), float(row[3]), float(row[4]), float(row[5]), float(row[6]), float(row[7]), float(row[8])]
				camera_matrixMat = [[float(row[0]), float(row[1]), float(row[2])], [float(row[3]), float(row[4]), float(row[5])], [float(row[6]), float(row[7]), float(row[8])]]		
			if(i == 4):
				distortion = [float(row[0]), float(row[1]), float(row[2]), float(row[3]), float(row[4])]	
			if(i == 5):
				camera_matrix2 = [float(row[0]), float(row[1]), float(row[2]), float(row[3]), float(row[4]), float(row[5]), float(row[6]), float(row[7]), float(row[8])]
				camera_matrixMat2 = [[float(row[0]), float(row[1]), float(row[2])], [float(row[3]), float(row[4]), float(row[5])], [float(row[6]), float(row[7]), float(row[8])]]			

	camera_matrix = np.array(camera_matrix)
	distortion = np.array(distortion)
	camera_matrix2 = np.array(camera_matrix2)
	camera_matrixMat = np.array(camera_matrixMat)
	camera_matrixMat2 = np.array(camera_matrixMat2)
	print(">> Camera Matrix:")
	print(camera_matrixMat)

	cam_info = CameraInfo()
	cam_info.height = height
	cam_info.width = width
	cam_info.distortion_model = "plumb_bob"
	cam_info.K = camera_matrix
	print(">> Camera Distortion:")
	print(distortion)
	cam_info.D = distortion.tolist()
	print("---------------------------------------------------------------------------")

	return cam_info, camera_matrix, camera_matrixMat
#	return cam_info, camera_matrix2, camera_matrixMat2
	
	

