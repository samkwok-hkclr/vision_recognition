#!/usr/bin/env python3 

import os
import sys
import random
from pathlib import Path

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

import cv2
import numpy as np
import open3d as o3d

from vision_recognition import ros_utils

print(sys.path)

from robotic_platform_msgs.srv import InitAlgo, GetSlotState, GetObjectPose
from MarketRestockingDep.AlgoGraspRestocking import AlgoGraspRestocking


class FakeRecognitionServer(Node):
    def __init__(self):
        super().__init__("fake_recognition_server")

        self.test_timer = self.create_timer(1.0, self.test_cb)

        self.init_algo_srv = self.create_service(InitAlgo, "init_algo", self.init_algo_cb)
        self.get_slot_state_srv = self.create_service(GetSlotState, "get_slot_state", self.get_slot_state_cb)
        self.get_object_pose_srv = self.create_service(GetObjectPose, "get_object_pose", self.get_object_pose_cb)

        self.get_logger().info("fake recognition server is started")

    def init_algo_cb(self, req, res):
        self.get_logger().info("init_algo service received")
        res.success = True
        return res
    
    def get_slot_state_cb(self, req, res):
        self.get_logger().info("get_slot_state service received")
        res.remain_qty = random.randint(0, 3)
        res.success = True
        return res
    
    def get_object_pose_cb(self, req, res):
        self.get_logger().info("get_slot_state service received")
        res.success = True
        return res

    def test_cb(self):
        algo_cfg_p = Path(get_package_share_directory("vision_recognition")) / "config" / "AlgoGraspRestockingDefault.json"

        algo = AlgoGraspRestocking(algo_cfg_p)
        algo.show_result = True                         # 关掉 pose 可视化开关
        algo.alg_cfg.detect_max = 10                    # 最大物体检测个数设定为10
        algo.save_vis_path = os.getcwd() + "/vision_data"  # 设定分割可视化结果保存路径
        target_object_id = 35                           # 设定待检测 sku id
        
        # 读取 RGB 和点云数据
        img_path = "/home/hkclr/robostore_ws/src/vision_recognition/marketvision_restocking_onnx/MarketRestockingDep/demo_data/img_14-29-47.png"
        pc_path = "/home/hkclr/robostore_ws/src/vision_recognition/marketvision_restocking_onnx/MarketRestockingDep/demo_data/ply_14-29-47.ply"
        pc = o3d.io.read_point_cloud(pc_path)
        pc_array = np.asarray(pc.points)
        img_array = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
        
        # 执行检测
        result = algo.detect(pc_array, img_array, target_object_id)

        # 获取检测出的物体 pose
        
        print("detected pose list:")
        print(result[0])

        print("pc_array type:" + str(type(pc_array)))
        print("img_array type:" + str(type(img_array)))
        
        self.test_timer.cancel()
        self.get_logger().info("test timer cancelled")
        self.get_logger().info(algo.save_vis_path)


def main(args=None):
    rclpy.init(args=args)
    node = FakeRecognitionServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()