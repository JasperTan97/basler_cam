from pypylon import pylon
import argparse
import time
import os
import cv2
from cv_bridge import CvBridge
import threading

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from basler_camera_interfaces.srv import TakePictures, ChangeFilename, StartVideo
from std_srvs.srv import Trigger


class CameraNode(Node):
    def __init__(self, camera_id: str):
        super().__init__(f'camera_{camera_id}')
        
        # Initialise camera
        self.camera = None
        subnet = "192.168.1"
        self.camera_id = camera_id
        self.camera_ip = subnet + '.' + camera_id
        tl_factory = pylon.TlFactory.GetInstance()
        for dev_info in tl_factory.EnumerateDevices():
            if self.camera_ip == dev_info.GetIpAddress():
                self.camera = pylon.InstantCamera(tl_factory.CreateDevice(dev_info))
                break
        else:
            self.get_logger().error(f"Camera {self.camera_ip} not connected.")
            return
        assert self.camera is not None
        self.camera.Open()
        self.camera.GainAuto.Value = "Continuous"
        self.camera.ExposureAuto.Value = "Continuous"
        
        self.file_directory = "images"
        os.makedirs(self.file_directory, exist_ok=True)
        file_name = f"camera_{self.camera_id}_{int(time.time())}"
        self.full_file_name = os.path.join(self.file_directory, file_name)
        
        self.get_logger().info(f"Camera {self.camera_id} initialised. All files will be saved in folder {self.full_file_name}.")
        
        # publisher
        self.image_publisher = self.create_publisher(Image, f'camera_{self.camera_id}/image',10)
        self.bridge = CvBridge()
        
        # service servers
        self.change_filename_server = self.create_service(
            ChangeFilename,
            f'camera_{self.camera_id}/change_filename',
            self.change_filename_cb
        )
        self.take_pictures_server = self.create_service(
            TakePictures,
            f'camera_{self.camera_id}/take_pictures',
            self.take_pictures_cb
        )
        self.start_video_server = self.create_service(
            StartVideo,
            f'camera_{self.camera_id}/start_video',
            self.start_video_cb
        )
        self.stop_video_server = self.create_service(
            Trigger,
            f'camera_{self.camera_id}/stop_video',
            self.stop_video_cb
        )
        
        self._video = False
        self._recording = False
        self._streaming = False
        self._cam_mutex = threading.Lock()
        
    def change_filename_cb(self, request, response):
        self.full_file_name = os.path.join(self.file_directory, request.file_name)
        self.get_logger().info(f"All Camera {self.camera_id} files will be saved in folder {self.full_file_name}.")
        response.success = True
        return response
    
    def take_pictures_cb(self, request, response):
        self.camera.StartGrabbingMax(request.frames)
        while self.camera.IsGrabbing():
            grabResult = self.camera.RetrieveResult(5000)
            if grabResult.GrabSucceeded():
                raw = grabResult.Array
                img = cv2.cvtColor(raw, cv2.COLOR_BAYER_RG2RGB)
                ts = int(time.time_ns())
                os.makedirs(self.full_file_name, exist_ok=True)
                fn = os.path.join(self.full_file_name, f"img_{ts}.png")
                cv2.imwrite(fn, img)
                grabResult.Release()
            else:
                response.success = False
                return response
        response.success = True
        return response
    
    def start_video_cb(self, request, response):
        if self._video:
            response.success = False
            response.message = 'Already recording'
            return response
        self._recording = request.record
        self._streaming = request.stream
        self.camera.AcquisitionFrameRateEnable.Value = True
        fps = request.fps
        if fps > 10.0:
            self.get_logger().warn(f"Desired FPS {fps} is over 10.0, and may not be achieved.")
            # we can get up to 23+ fps by shortening exposure time
        self.camera.AcquisitionFrameRate.Value = fps
        if self._recording:
            ts = time.time_ns()
            os.makedirs(self.full_file_name, exist_ok=True)
            fname = os.path.join(self.full_file_name, f'record_{ts}.avi')
            width = int(self.camera.Width.GetValue())
            height = int(self.camera.Height.GetValue())
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            self._writer = cv2.VideoWriter(fname, fourcc, fps, (width, height))
        
        self._video = True
        self.camera.StartGrabbing(pylon.GrabStrategy_OneByOne)
        self._thread = threading.Thread(target=self._record_loop, daemon=True)
        self._thread.start()
        
        response.success = True
        response.message = f"Video started"
        return response
    
    def stop_video_cb(self, request, response):
        if not self._video:
            response.success = False
            response.message = 'Not currently recording'
            return response

        with self._cam_mutex:
            if self.camera.IsGrabbing():
                self.camera.StopGrabbing()
        self._thread.join(timeout=5.0)
        if self._recording:
            self._writer.release()
            self._writer = None

        self._video = False
        self._recording = False
        self._streaming = False

        response.success = True
        response.message = 'Recording stopped'
        return response
    
    def _record_loop(self):
        """Grabs frames until self._recording is False."""
        while self._video and self.camera.IsGrabbing():
            with self._cam_mutex:
                grab = self.camera.RetrieveResult(
                    5000, pylon.TimeoutHandling_ThrowException)
                if grab.GrabSucceeded():
                    raw = grab.Array 
                    frame = cv2.cvtColor(raw, cv2.COLOR_BAYER_RG2RGB)
                    if self._recording:
                        self._writer.write(frame)
                    if self._streaming:
                        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                        img_msg.header.stamp = self.get_clock().now().to_msg()
                        img_msg.header.frame_id = f'camera_{self.camera_id}'
                        self.image_publisher.publish(img_msg)
                        
                grab.Release()
    
        
def main(args=None):
    parser = argparse.ArgumentParser(description="Run Basler camera node")
    parser.add_argument(
        "--camera_id",
        required=True,
        help="IP address of the Basler camera to connect to"
    )
    args, ros_args = parser.parse_known_args()
    
    rclpy.init(args=ros_args)
    node = CameraNode(args.camera_id)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()