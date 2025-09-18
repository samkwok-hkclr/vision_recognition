#!/usr/bin/env python3 

import os
import sys
import random
from collections.abc import Callable
from datetime import datetime
from pathlib import Path
from functools import wraps

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

import cv2
import numpy as np
import open3d as o3d

from robotic_platform_msgs.msg import ObjectPose
from robotic_platform_msgs.srv import InitAlgo, GetSlotState, GetObjectPose

from vision_recognition import ros_utils
from MarketRestockingDep.AlgoGraspRestocking import AlgoGraspRestocking

class RecognitionServer(Node):
    def __init__(self):
        super().__init__("recognition_server")
        self.logger = self.get_logger()

        self.declare_parameter("folder_name", "vision_data")
        self.declare_parameter("ws_name", "robostore_ws")

        self.folder_name_ = self.get_parameter('folder_name').get_parameter_value().string_value
        self.ws_name_ = self.get_parameter('ws_name').get_parameter_value().string_value

        self.algo = None
        self.test_timer = self.create_timer(1.0, self.test_cb)
        self.init_timer = self.create_timer(1.0, self.init_cb)

        self.is_initialized = False

        self.init_algo_srv = self.create_service(InitAlgo, "init_algo", self.init_algo_cb)
        self.get_slot_state_srv = self.create_service(GetSlotState, "get_slot_state", self.get_slot_state_cb)
        self.get_object_pose_srv = self.create_service(GetObjectPose, "get_object_pose", self.get_object_pose_cb)

        self.logger.info("Recognition Server is started")

    #======Callbacks corresponding to each of the ROS services==============================
    def wrapper(callback: Callable):
        """
        wraps around a callback to
        1. record time needed
        2. swallow all exception, always reply client
        """
        @wraps(callback)
        def _cb(self, req, res):            
            start = datetime.now()
            res.success = False

            try:
                res = callback(self, req, res)
            except:
                res.success = False                
                import traceback
                self.logger.error(traceback.format_exc())                
        
            self.logger.info(f"<<< Request served : {datetime.now() - start} s")
            return res

        return _cb

    @wrapper
    def get_object_pose_cb(self, req: GetObjectPose.Request, res: GetObjectPose.Response):
        self.logger.info("get_object_pose service received")
        if not self.is_initialized:
            self.logger.warn("Service not initialized")
            return res

        img_arr, ply_arr = self.extract_img_ply_from_request(req)

        self.logger.debug(f"ply_arr type: {str(type(ply_arr))}")
        self.logger.debug(f"img_arr type: {str(type(img_arr))}")

        result = self.algo.detect(ply_arr, img_arr, req.target_object_id)

        if result is not None and len(result) > 0:
            poses = result[0]

            self.logger.debug(f"detected pose list: {result[0]}")
            self.logger.debug(f"detected pose list: {result}")

            for pose in poses:
                msg = ObjectPose()
                msg.pose = ros_utils.get_pose_from_array(pose)
                msg.object_id = req.target_object_id
                res.object_poses.append(msg)

            self.logger.debug(f"number of detected pose(s): {len(poses)}")
        
        res.success = True
        return res

    @wrapper
    def init_algo_cb(self, req, res):
        self.logger.info("init_algo service received")
        res.success = True
        return res
    
    @wrapper
    def get_slot_state_cb(self, req, res):
        self.logger.info("get_slot_state service received")
        res.remain_qty = random.randint(0, 3)
        res.success = True
        return res
        
    def init_cb(self):
        algo_cfg_p = os.path.join(
            get_package_share_directory("vision_recognition"), "config", "AlgoGraspRestockingDefault.json")
        self.algo = AlgoGraspRestocking(algo_cfg_p)
        # self.algo._set_vis_path(self.get_save_folder(), self.get_save_folder())
        self.algo.show_result = True
        self.algo.alg_cfg.detect_max = 10
        # self.algo.logger = self.logger

        if self.algo is None:
            self.get_logger.error("algo init error")
            return

        self.is_initialized = True

        self.logger.info("algo is up!")
        self.init_timer.cancel()

    def get_save_folder(self) -> str:
        home = os.getenv("HOME")
        if home is None:
            return "/vision_foler"

        ws_name = os.getenv("WS_NAME")

        if self.ws_name_ != "":
            path = Path(home) / self.ws_name_ / self.folder_name_
        elif ws_name is not None:
            path = Path(home) / ws_name / self.folder_name_
        else:
            path = Path(home) / self.folder_name_
            
        self.logger.info(f"history_data path: {str(path)}")
        return str(path)

    def test_cb(self):
        if not self.is_initialized:
            return

        self.logger.debug(f"alive")


    def extract_img_ply_from_request(self, request):
        """
        get img & ply from 
        a) the SHM handle passed from the request OR
        b) ROS msg data fields when a) fails
        """
        # img = ros_utils.msg_to_cv2(request.image, request.image.encoding)
        # FIXME: image encoding is mismatch.
        img = ros_utils.msg_to_cv2(request.image, request.image.encoding)
        ply_arr = ros_utils.pointcloud2_to_array(request.pointcloud)
        
        return img, ply_arr

def main(args=None):
    rclpy.init(args=args)
    node = RecognitionServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# target_object_id = 35 # 设定待检测 sku id

# # 读取 RGB 和点云数据
# img_path = "/home/hkclr/robostore_ws/vision_data/2025_08_11/image_17_11_40.png"
# pc_path = "/home/hkclr/robostore_ws/vision_data/2025_08_11/pc_17_11_40.ply"
# pc = o3d.io.read_point_cloud(pc_path)
# pc_array = np.asarray(pc.points)
# img_array = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)

# # 执行检测
# result = self.algo.detect(pc_array, img_array, target_object_id)

# # 获取检测出的物体 pose

# print("detected pose list:")
# print(result)

# print("pc type:" + str(type(pc)))
# print("pc.points type:" + str(type(pc.points)))
# print("pc_array type:" + str(type(pc_array)))
# print("img_array type:" + str(type(img_array)))

# self.test_timer.cancel()
# self.logger.info("test timer cancelled")
# self.logger.info(str(self.algo.save_vis_path))