
import random

import rclpy
from rclpy.node import Node

from robotic_platform_msgs.srv import InitAlgo, GetSlotState, GetObjectPose

class FakeRecognitionServer(Node):
    def __init__(self):
        super().__init__("fake_recognition_server")

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


def main(args=None):
    rclpy.init(args=args)
    node = FakeRecognitionServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()