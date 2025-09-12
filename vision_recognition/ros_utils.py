import time

import numpy as np

import cv2
from cv_bridge import CvBridge 

from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2, PointField

bridge = CvBridge()

def cv2_to_msg(img):
    return bridge.cv2_to_imgmsg(img)

def msg_to_cv2(msg, encoding):
    return bridge.imgmsg_to_cv2(msg, encoding)

DUMMY_FIELD_PREFIX = '__'
type_mappings = [
    (PointField.INT8, np.dtype('int8')), 
    (PointField.UINT8, np.dtype('uint8')), 
    (PointField.INT16, np.dtype('int16')),
    (PointField.UINT16, np.dtype('uint16')), 
    (PointField.INT32, np.dtype('int32')), 
    (PointField.UINT32, np.dtype('uint32')),
    (PointField.FLOAT32, np.dtype('float32')), 
    (PointField.FLOAT64, np.dtype('float64'))
]
pftype_to_nptype = dict(type_mappings)
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)
# sizes (in bytes) of PointField types
pftype_sizes = {
    PointField.INT8: 1, 
    PointField.UINT8: 1, 
    PointField.INT16: 2, 
    PointField.UINT16: 2,
    PointField.INT32: 4,
    PointField.UINT32: 4, 
    PointField.FLOAT32: 4, 
    PointField.FLOAT64: 8
}

def fields_to_dtype(fields, point_step):
    '''Convert a list of PointFields to a numpy record datatype.
    '''
    offset = 0
    np_dtype_list = []
    for f in fields:
        while offset < f.offset:
            # might be extra padding between fields
            np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        dtype = pftype_to_nptype[f.datatype]
        if f.count != 1:
            dtype = np.dtype((dtype, f.count))

        np_dtype_list.append((f.name, dtype))
        offset += pftype_sizes[f.datatype] * f.count

    # might be extra padding between points
    while offset < point_step:
        np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
        offset += 1
        
    return np_dtype_list

def get_each(xyzi):
    return xyzi[0], xyzi[1], xyzi[2]

def get_array(cloud_arr):
    vfunc = np.vectorize(get_each)
    x_arr, y_arr, z_arr = vfunc(cloud_arr)
    print("- 4:", time.time())

    return np.column_stack((x_arr, y_arr, z_arr))

def pointcloud2_to_array(cloud_msg):
    ''' Converts a rospy PointCloud2 message to a numpy recordarray 
    
    Reshapes the returned array to have shape (height, width), even if the height is 1.

    The reason for using np.frombuffer rather than struct.unpack is speed... especially
    for large point clouds, this will be <much> faster.
    '''
    # construct a numpy record type equivalent to the point type of this cloud
    dtype_list = fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)
    #print("- 1:", time.time())

    # parse the cloud into an array
    cloud_arr = np.frombuffer(cloud_msg.data, dtype_list)
    #print("- 2:", time.time())

    # remove the dummy fields that were added
    cloud_arr = cloud_arr[
        [fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]
    #print("- 3:", time.time())

    cloud_arr = cloud_arr.view('<f4').reshape(len(cloud_arr), -1)
    return cloud_arr[:,:3]

def pcl_to_pointcloud2(cloud, stamp=None, frame_id=None):
    '''Converts a pcl point cloud to a sensor_msgs.msg.PointCloud2.
    ref: http://docs.ros.org/en/jade/api/ros_numpy/html/point__cloud2_8py_source.html
    '''
    cloud_arr = cloud.to_array()
    nChannel = cloud_arr.shape[1]
    assert nChannel == 3
    assert cloud_arr.dtype == np.float32

    cloud_msg = PointCloud2()

    if stamp is not None:
        cloud_msg.header.stamp = stamp
    if frame_id is not None:
        cloud_msg.header.frame_id = frame_id
    cloud_msg.height = cloud.height
    cloud_msg.width = cloud.width
    
    cloud_msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
    ]
    
    cloud_msg.is_bigendian = False # assumption
    cloud_msg.point_step = cloud_arr.dtype.itemsize * nChannel
    cloud_msg.row_step = cloud_msg.point_step*cloud.width
    cloud_msg.is_dense = cloud.is_dense #all([np.isfinite(cloud_arr[fname]).all() for fname in cloud_arr.dtype.names])
    cloud_msg.data = cloud_arr.tobytes()
    return cloud_msg

def get_pose_from_array(pose_arr):
    """
    pose_arr format: [x, y, z, qx, qy, qz, qw]
    x : x component of the translation
    qx: x component of the quaterion
    """
    pose = Pose()
    pose.position.x = pose_arr[0]
    pose.position.y = pose_arr[1]
    pose.position.z = pose_arr[2]
    pose.orientation.x = pose_arr[3]
    pose.orientation.y = pose_arr[4]
    pose.orientation.z = pose_arr[5]
    pose.orientation.w = pose_arr[6]
    return pose