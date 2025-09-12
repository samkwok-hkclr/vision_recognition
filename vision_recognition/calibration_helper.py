import os
import sys
import csv
from datetime import datetime
from typing import Optional, List, Final
from threading import Lock
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import Empty
from geometry_msgs.msg import Pose

from robotic_platform_msgs.msg import CameraDataType
from robotic_platform_msgs.srv import SaveCameraData

from vision_recognition import ros_utils

class CalibrationHelper(Node):
    def __init__(self, executor):
        super().__init__("calibration_helper")
        self.logger = self.get_logger()
        self.executor_ = executor

        self.declare_parameter("folder_name", "calibration_data")
        self.declare_parameter("ws_name", "robostore_ws")
        self.declare_parameter("camera_id", 1)

        self.camera_id_: Final[int] = self.get_parameter('camera_id').get_parameter_value().integer_value
        self.ws_name_ = self.get_parameter('ws_name').get_parameter_value().string_value
        self.folder_name_ = self.get_parameter('folder_name').get_parameter_value().string_value
        self.path_ = self.get_calibration_folder()
        self.file_ = datetime.now().strftime("%H_%M_%S") + ".csv"
        self.full_path_: Final[str] = str(Path(self.path_) / self.file_)

        self.mutex = Lock()

        self.exit_flag_ = False
        self.curr_pose_: Pose = Pose()

        self.pose_sub_ = self.create_subscription(Pose, "current_pose", self.pose_cb, 10)
        self.move_sub_ = self.create_subscription(Empty, "next_pose", self.next_step_cb, 10)

        self.save_camera_cli_ = self.create_client(SaveCameraData, "save_camera_data", callback_group=MutuallyExclusiveCallbackGroup())

        self.logger.info(f"Press Ctrl+C to exit")
        self.logger.info(f"camera ID: {self.camera_id_}")
        self.logger.info(f"calibration folder: {self.path_}")

    def next_step_cb(self, msg: Empty) -> None:
        if not self.save_camera_cli_.wait_for_service(timeout_sec=2.0):
            self.logger.warn("Service unavailable. Skipping capture.")
            return
 
        tmp = list()
        with self.mutex:
            tmp.append(self.curr_pose_.position.x)
            tmp.append(self.curr_pose_.position.y)
            tmp.append(self.curr_pose_.position.z)
            tmp.append(self.curr_pose_.orientation.x)
            tmp.append(self.curr_pose_.orientation.y)
            tmp.append(self.curr_pose_.orientation.z)
            tmp.append(self.curr_pose_.orientation.w)

        self.write_to_csv(tmp)

        req = SaveCameraData.Request()
        req.camera_id = self.camera_id_
        req.type = CameraDataType.IMAGE

        def _future_done(future):
            self.logger.info("captured image")

        try:
            future = self.save_camera_cli_.call_async(req)
            future.add_done_callback(
                lambda f: self.logger.info("Capture succeeded.") 
                if f.result().success 
                else self.logger.error("Capture failed!")
            )
            self.logger.info("in_mtrl_box_cli is called, waiting for future done")

        except Exception as e:
            self.logger.error(f"Service call failed: {str(e)}")

        
    def pose_cb(self, msg: Pose) -> None:
        with self.mutex:
            self.curr_pose_ = msg
            self.logger.debug(str(self.curr_pose_))

    def get_calibration_folder(self) -> str:
        home = os.getenv("HOME")
        if home is None:
            return "/calibration_data"

        ws_name = os.getenv("WS_NAME")

        if self.ws_name_ != "":
            path = Path(home) / self.ws_name_ / self.folder_name_
        elif ws_name is not None:
            path = Path(home) / ws_name / self.folder_name_
        else:
            path = Path(home) / self.folder_name_

        return str(path)

    def create_csv(self) -> Optional[bool]:
        try:
            with open(self.full_path_, "w") as fd:
                pass 
        except Exception as e:
            self.logger.error("create file error")
            return False
        return True

    def write_to_csv(self, pose: List[float]) -> None:
        if len(pose) != 7:
            self.logger.error(f"Pose array length: {len(pose)}")
            return

        write_str = ",".join(map(str, pose))
        self.logger.info(f"Pose: {write_str}")
        try:
            os.makedirs(self.path_, exist_ok=True)
            with open(self.full_path_, "a+") as fd:
                fd.write(write_str)
                fd.write("\n")
        except PermissionError:
            self.logger.error(f"Permission denied: {self.full_path_}")
        except Exception as e:
            self.logger.error(f"Write failed: {str(e)}")
    

def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    node = CalibrationHelper(executor)

    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.logger.info('Keyboard interrupt, shutting down.\n')

    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()