#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import std_msgs.msg
import depthai as dai


#================================================================================
# Left and Right camera definition
#================================================================================


def left_Camera_Defitions(pipeline):

	#______________________ Define a source - Left camera ______________________
	camLeft = pipeline.createMonoCamera()
	camLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
	camLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
#	camLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

	return camLeft


def right_Camera_Defitions(pipeline):

	#______________________ Define a source - Right camera ______________________
	camRight = pipeline.createMonoCamera()
	camRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
	camRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
#	camRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

	return camRight
	

def right_cam_definition(pipeline, camRight):

	xoutRight = pipeline.createXLinkOut()
	xoutRight.setStreamName('right')
	camRight.out.link(xoutRight.input)

#================================================================================
# Depth Map definition
#================================================================================

	
def depth_definition(pipeline, camLeft, camRight):

	print("---------------------------------------------------------------------------")
	print("...Depth Stereo Definition...")
	depth = pipeline.createStereoDepth()
	depth.setOutputDepth(True)
#	depth.setOutputRectified(True)
#	depth.setSubpixel(True)

#	depth.setOutputRectified(True)  # The rectified streams are horizontally mirrored by default
#	depth.setConfidenceThreshold(255)
#	depth.setRectifyEdgeFillColor(0)  # Black, to better see the cutout from rectification (black stripe on the edges)



	depth.setConfidenceThreshold(200)
	#_______________________________ Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default) _______________________________
	median = dai.StereoDepthProperties.MedianFilter.KERNEL_7x7  # For depth filtering
	#median = dai.StereoDepthProperties.MedianFilter.KERNEL_5x5  # For depth filtering
	depth.setMedianFilter(median)

	camLeft.out.link(depth.left)
	camRight.out.link(depth.right)

	#_______________________________ Create output _______________________________
	xout = pipeline.createXLinkOut()
	xout.setStreamName("depth")
	depth.depth.link(xout.input)
##	xout.setStreamName("disparity")
##	depth.disparity.link(xout.input)
	print("---------------------------------------------------------------------------")


