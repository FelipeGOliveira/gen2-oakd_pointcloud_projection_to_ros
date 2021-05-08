#!/usr/bin/env python3
import numpy as np
import struct
import rospy
import ctypes
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


#================================================================================
# Code from pointcloud2.py
#================================================================================

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

def _get_struct_fmt(is_bigendian, fields, field_names=None):
	fmt = '>' if is_bigendian else '<'

	offset = 0
	for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
		if offset < field.offset:
			fmt += 'x' * (field.offset - offset)
			offset = field.offset
		if field.datatype not in _DATATYPES:
			print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
		else:
			datatype_fmt, datatype_length = _DATATYPES[field.datatype]
			fmt += field.count * datatype_fmt
			offset += field.count * datatype_length

	return fmt


def create_cloud(fields, points):
	cloud_struct = struct.Struct(_get_struct_fmt(False, fields))

	buff = ctypes.create_string_buffer(cloud_struct.size * len(points))

	point_step, pack_into = cloud_struct.size, cloud_struct.pack_into
	offset = 0
	for p in points:
		pack_into(buff, offset, *p)
		offset += point_step

	return 1, len(points), cloud_struct.size, cloud_struct.size * len(points), buff.raw
	# height, width, point_step, row_step, data




#================================================================================
# Create PointCloud2 message
#================================================================================

def createPointCloudData(frame2d, invK):
	points = []
		
	x1 = np.arange(frame2d.shape[0]).repeat(frame2d.shape[1],axis=0 )
	x1 = x1 * invK[0,0]

	y1 = np.arange(frame2d.shape[1]).reshape(1, frame2d.shape[1]).repeat(frame2d.shape[0],axis=0).reshape(frame2d.size)
	y1 = y1 * invK[1,1]

	z = frame2d.flatten()

#	c = frameRight.flatten()
#	rgb = [ struct.unpack('I', struct.pack('BBBB', val, val, val, val) )[0] for val in c ]

	o1 = np.full(frame2d.size, invK[0,2])
	o2 = np.full(frame2d.size, invK[1,2])
	X = ( x1 + ( o1 ) ) * z
	Y = ( y1 + ( o2 ) ) * z

	points = list(zip(X, Y, z)) 		# , rgb))

	return points


def publishPointCloud(stamp, points, pointcloud_pub):	
	
	msg_pf1 = PointField()
	msg_pf1.name = 'x'
	msg_pf1.offset = 0
	msg_pf1.datatype = 7
	msg_pf1.count = 1

	msg_pf2 = PointField()
	msg_pf2.name = 'y'
	msg_pf2.offset = 4
	msg_pf2.datatype = 7
	msg_pf2.count = 1

	msg_pf3 = PointField()
	msg_pf3.name = 'z'
	msg_pf3.offset = 8
	msg_pf3.datatype = 7
	msg_pf3.count = 1

	"""
	msg_pf4 = PointField()
	msg_pf4.name = 'rgb'
	msg_pf4.offset = 12
	msg_pf4.datatype = PointField.UINT32
	msg_pf4.count = 1
	"""

	fields = [msg_pf1, msg_pf2, msg_pf3]	#, msg_pf4]
	height, width, point_step, row_step, data = create_cloud(fields, points)

	cloud_msg = PointCloud2()
	cloud_msg.header.stamp = stamp
	cloud_msg.header.frame_id = "map"
	cloud_msg.height = height
	cloud_msg.width = width
	cloud_msg.fields = [msg_pf1, msg_pf2, msg_pf3]	#, msg_pf4]
	cloud_msg.is_bigendian = False
	cloud_msg.point_step =	point_step
	cloud_msg.row_step = row_step
	cloud_msg.data = data
	pointcloud_pub.publish(cloud_msg)


