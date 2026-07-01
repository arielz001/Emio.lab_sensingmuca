import os
import json
import logging
from time import sleep
import tkinter as tk
from tkinter import ttk
import sys

import numpy as np
import cv2 as cv
import pyrealsense2 as rs
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from _camerafeedwindow import CameraFeedWindow
from PIL import Image, ImageTk
from pupil_apriltags import Detector

FORMAT = "[%(levelname)s]\t[%(filename)s:%(lineno)s - %(funcName)s() ] %(message)s"
logging.basicConfig(format=FORMAT, level=logging.INFO)
logger = logging.getLogger(__name__)

CONFIG_FILENAME = os.path.dirname(__file__) + '/cameraparameter.json'


def convert_depth_pixel_to_metric_coordinate(depth, pixel_x, pixel_y, camera_intrinsics):
    """
    Convert the depth and image point information to metric coordinates
    """
    X = (pixel_x - camera_intrinsics.ppx) / camera_intrinsics.fx * depth
    Y = (pixel_y - camera_intrinsics.ppy) / camera_intrinsics.fy * depth
    return [X, Y, depth]


def compute_cdg(contour):
    M = cv.moments(contour)
    cX = 0
    cY = 0
    if M['m00'] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    return cX, cY


def listCameras() -> list:
    context = rs.context()
    return [d.get_info(rs.camera_info.serial_number) for d in context.devices]


class DepthCamera:

    height = 480
    width = 640
    device = None
    pipeline_profile = None
    pipeline_wrapper = None
    config = None
    pipeline = None
    point_cloud = None
    intr = None
    profile = None
    initialized = False
    pc = None
    compute_point_cloud = False
    tracking = True
    trackers_pos = []
    trackers_leg = []
    overlayWindow = None
    overlayFrame = None
    rootWindow = None

    @property
    def camera_serial(self) -> str:
        return self.device.get_info(rs.camera_info.serial_number)

    def __init__(self, camera_serial: str=None, compute_point_cloud: bool=False, show_video_feed: bool=False, tracking: bool=True) -> None:
        self.tracking = tracking
        self.show_video_feed = show_video_feed
        self.compute_point_cloud = compute_point_cloud
        self.overlayWindow = None  
        self.rootWindow = None

        self.initialized = True
        self.init_realsense(camera_serial)
        if not self.initialized:
            return

        self.pc = rs.pointcloud()
        self.trackers_pos = []
        self.trackers_leg = []

        if self.show_video_feed:
            self.createWindows()

        # self.detector = Detector(
        #     families='tag36h11',
        #     nthreads=4,
        #     quad_decimate=1.0,    # Forces native resolution to avoid losing far/tilted edges
        #     quad_sigma=0.0,       # Smooths binary artifacts caused by CLAHE contrast
        #     refine_edges=1,       # Ultra-precise subpixel corner refinement
        #     decode_sharpening=0.0,
        #     debug=0
        # )
        self.detector = Detector(
            families='tag36h11', # O la familia exacta que sea tu tag
            nthreads=4,
            quad_decimate=1.0,     # PRO TIP: 1.0 deshabilita el submuestreo. Busca a resolución completa.
            quad_sigma=0.8,        # Aplica un leve desenfoque interno para suavizar el ruido de los píxeles
            refine_edges=1,        # Activa el refinamiento estricto de bordes analizando gradientes
            decode_sharpening=0.1,# Fuerza el enfoque de los bits internos antes de decodificar
            debug=0

        )

        self.update()  # Get the first frame

    def createWindows(self):
        self.rootWindow = tk.Tk()
        self.rootWindow.withdraw()  

        self.createOverlayWindow()

        self.rootWindow.protocol("WM_DELETE_WINDOW", self.quit)
        self.rootWindow.update_idletasks()

    def createOverlayWindow(self):
        if self.overlayWindow is None or not getattr(self.overlayWindow, 'running', False):
            self.overlayWindow = CameraFeedWindow(
                rootWindow=self.rootWindow,
                name='Realsense'  
            )
    
    def quit(self):
        self.overlayWindow.closed()
        self.rootWindow.destroy()
        self.show_video_feed = False
        self.rootWindow = None

    def init_realsense(self, camera_serial=None):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        if camera_serial is not None:
            self.config.enable_device(camera_serial)

        # Get device product line for setting a supporting resolution
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        try:
            self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        except Exception as err:
            self.initialized = False
            raise Exception('DepthCamera', str(err))

        self.device = self.pipeline_profile.get_device()

        self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, 30)

        depth_sensor = self.device.first_depth_sensor()
        depth_sensor.set_option(rs.option.depth_units, 0.001)

        cfg = self.pipeline.start(self.config)

        self.profile = cfg.get_stream(rs.stream.depth)
        self.intr = self.profile.as_video_stream_profile().get_intrinsics()


    def corregir_gamma(self, image, gamma=0.5): 
        invGamma = 1.0 / gamma
        tabla = np.array([((i / 255.0) ** invGamma) * 255 
                        for i in np.arange(0, 256)]).astype("uint8")
        return cv.LUT(image, tabla)
    

    def clahe_effect(self, image):
        if image.dtype != np.uint8:
            image = cv.normalize(image, None, 0, 255, cv.NORM_MINMAX).astype(np.uint8)
        if len(image.shape) == 3 and image.shape[2] == 3:
            lab = cv.cvtColor(image, cv.COLOR_BGR2LAB)
            l_channel, a_channel, b_channel = cv.split(lab)
            clahe = cv.createCLAHE(clipLimit=4.0, tileGridSize=(4,4))
            cl = clahe.apply(l_channel)
            limg = cv.merge((cl, a_channel, b_channel))
            resultado = cv.cvtColor(limg, cv.COLOR_LAB2BGR)
        else:
            clahe = cv.createCLAHE(clipLimit=4.0, tileGridSize=(4,4))
            resultado = clahe.apply(image)
        return resultado



    def get_frame(self):
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()
        aligned_frame = rs.align(rs.stream.color).process(frames)
        intrinsics = self.intr

        depth_frame = aligned_frame.get_depth_frame()
        color_frame = aligned_frame.get_color_frame()

        if not depth_frame or not color_frame:
            logger.debug('no frame')
            return False, color_frame, depth_frame

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        return True, color_image, depth_image, depth_frame, intrinsics


    def detect_tags(self, frame, intrinsics):
        """
        Detects AprilTags in the frame, estimates 3D poses, and draws axes/borders.
        """
        overlay = frame.copy()
        # corrected_overlay = self.corregir_gamma(overlay, gamma=5.0)
        corrected_overlay = self.clahe_effect(overlay)
        corrected_overlay[:,:int(overlay.shape[1]//2.5)] = overlay[:,:int(overlay.shape[1]//2.5)]
        overlay2 = corrected_overlay
        overlay2 = cv.detailEnhance(overlay2, sigma_s=5, sigma_r=0.2)
        # overlay = cv.edgePreservingFilter(overlay, flags=1, sigma_s=64, sigma_r=0.2)
        # overlay = corrected_overlay
        gray = cv.cvtColor(overlay2, cv.COLOR_BGR2GRAY)
        
        camera_params = [intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy]
        tag_size = 0.015  # 1.5 cm 

        # Pose detection execution
        result = self.detector.detect(
            img=gray,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=tag_size
        )

        # Set up camera matrix for OpenCV projections
        K = np.array([[camera_params[0], 0, camera_params[2]],
                      [0, camera_params[1], camera_params[3]],
                      [0, 0, 1]], dtype=np.float32)
        
        axis_3d = np.float32([[tag_size, 0, 0], [0, tag_size, 0], [0, 0, -tag_size], [0, 0, 0]]).reshape(-1, 3)

        for det in result:
            R = det.pose_R
            t = det.pose_t
            
            # Draw tag borders
            for i in range(4):
                pt1 = tuple(det.corners[i].astype(int))
                pt2 = tuple(det.corners[(i + 1) % 4].astype(int))
                cv.line(overlay, pt1, pt2, (0, 255, 0), 2)

            # 3D Axis Projection
            rvec, _ = cv.Rodrigues(R)
            imgpts, _ = cv.projectPoints(axis_3d, rvec, t, K, distCoeffs=None)
            imgpts = imgpts.astype(int).squeeze()
            
            center_of_apriltag = tuple(imgpts[3]) 
            cv.line(overlay, center_of_apriltag, tuple(imgpts[0]), (0, 0, 255), 2)  # X: RED
            cv.line(overlay, center_of_apriltag, tuple(imgpts[1]), (0, 255, 0), 2)  # Y: GREEN
            cv.line(overlay, center_of_apriltag, tuple(imgpts[2]), (255, 0, 0), 2)  # Z: BLUE
            
            # Draw tag ID text
            cv.putText(overlay, str(det.tag_id), (int(det.center[0]) - 5, int(det.center[1]) + 5),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        return result, overlay

    def update(self):
        ret, frame, depth_image, depth_frame, intrinsics = self.get_frame()

        if ret is False or frame is None:
            return

        self.Frame = frame

        # -----------------------------------------
        # clahe processing maybe
        # -----------------------------------------
        # self.Frame = cv.convertScaleAbs(self.Frame, alpha=1.3, beta=40)

        # ================================================================
        # Calling tag detection method
        # ================================================================
        result, overlay = self.detect_tags(self.Frame, intrinsics)

        # Update tracker metrics based on detection if tracking is active
        if self.tracking:
            self.trackers_pos = []  # Reset tracker lists
            self.trackers_leg = []

            for det in result:
                t = det.pose_t
                tx, ty, tz = t[0][0], t[1][0], t[2][0]
                # print(f"det.tag_id: {det.tag_id}")
                # Convert positions into millimeters
                if det.tag_id == 1 or det.tag_id == 3:
                    self.trackers_pos.append([tx * 1000, ty * 1000, tz * 1000])
                elif det.tag_id == 2:
                    self.trackers_leg.append([tx * 1000, ty * 1000, tz * 1000])


        overlay = cv.rotate(overlay, cv.ROTATE_90_COUNTERCLOCKWISE)

        if overlay is not None and self.overlayWindow is not None:
            self.overlayWindow.set_frame(overlay)

        if self.compute_point_cloud:
            points = self.pc.calculate(depth_frame)
            v = points.get_vertices()
            self.point_cloud = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz

        if self.show_video_feed:
            if self.rootWindow is None:
                self.createWindows()
            if self.overlayWindow.running:
                self.overlayWindow.set_frame(overlay)
            self.rootWindow.update()

    def close(self):
        if self.pipeline:
            self.pipeline.stop()
        if self.rootWindow:
            self.rootWindow.destroy()
        self.initialized = False

    def run_loop(self):
        while True:
            if self.rootWindow is None or not self.rootWindow.winfo_exists():
                break
            if self.show_video_feed:
                self.rootWindow.update()
            self.update()

        self.close()