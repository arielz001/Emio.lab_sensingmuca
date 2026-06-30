import threading
import logging
import os 
import sys 

import numpy as np

import _depthcamera_tag as depthcamera

FORMAT = "[%(levelname)s]\t[%(filename)s:%(lineno)s - %(funcName)s() ] %(message)s"
logging.basicConfig(format=FORMAT, level=logging.INFO)
logger = logging.getLogger(__name__)




class EmioCamera:
    """
    A class to interface with the Realsense camera on Emio.
    This class opens the camera in the same process as the code is running from.

    It is recommendend to use this class if you want to use the camera in a SOFA scene.

    :::warning
    If you want to open the camera in another process, you can use the [MultiprocessEmioCamera](#MultiprocessEmioCamera) class.
    :::


    Example:
        ```python
        from emioapi import EmioCamera

        # Create an instance of EmioCamera
        camera = EmioCamera(show=True, track_markers=True, compute_point_cloud=True)

        # Open the camera
        if camera.open():
            try:
                while camera.is_running:
                    # Update camera frames and tracking
                    camera.update()

                    # Access tracker positions
                    positions = camera.trackers_pos
                    print("Tracker positions:", positions)

                    # Access point cloud data
                    pc = camera.point_cloud
                    print("Point cloud shape:", pc.shape)

                    # Access HSV and mask frames
                    hsv = camera.hsv_frame
                    mask = camera.mask_frame

                    # ... (process frames as needed)

                    # For demonstration, break after one iteration
                    break
            finally:
                # Close the camera when done
                camera.close()
        ```

    
    """
    _lock = threading.Lock()
    _compute_point_cloud: bool = False
    _camera: depthcamera.DepthCamera = None
    _tracking: bool = True
    _running: bool = False
    _parameter: dict = None
    _trackers_pos: list = []
    _trackers_leg: list = []
    _point_cloud: np.ndarray = None

    camera_serial: str = None


    def __init__(self, camera_serial=None, parameter=None, show=False, track_markers=True, compute_point_cloud=False):
        """
        Initialize the camera.
        Args:
            camera_serial: str: The serial number of the camera to connect to. If None, the first camera found will be used.
            parameter: dict:  The camera parameters. If None, the lastest save paramters are used from a file, but if no file is found, default values will be used.
            show: bool:  Whether to show the camera HSV and Mask frames or not.
            track_markers: bool:  Whether to track objects or not.
            compute_point_cloud: bool: Whether to compute the point cloud or not.
        """
        self.camera_serial = camera_serial
        self._tracking = track_markers
        self._show = show
        self._compute_point_cloud = compute_point_cloud
        if parameter is not None:
            self._parameter = parameter



    ##########################
    #  PROPERTIES
    ##########################



    @property
    def is_running(self) -> bool:
        """
        Get the running status of the camera.
        Returns:
            bool: The running status of the camera.
        """
        return self._running
    

    @property
    def track_markers(self) -> bool:
        """
        Get whether the camera is tracking objects or not.
        Returns:
            bool: True if the camera is tracking the markers, else False.
        """
        return self._tracking
    

    @track_markers.setter
    def track_markers(self, value: bool):
        """
        Set the tracking status of the camera.
        Args:
            value: bool: The new tracking status.
        """
        self._tracking = value

    @property
    def compute_point_cloud(self) -> bool:
        """
        Get whether the camera is computing the point cloud or not.
        Returns:
            bool: True if the camera is computing the point cloud, else False.
        """
        return self._compute_point_cloud
    

    @compute_point_cloud.setter
    def compute_point_cloud(self, value: bool):
        """
        Set the point cloud computation status of the camera.
        Args:
            value: bool: The new point cloud computation status.
        """
        self._compute_point_cloud = value

    
    @property
    def show_frames(self) -> bool:
        """
        Get whether the camera HSV and mask frames are shown in windows.
        Returns:
            bool: The show status of the camera.
        """
        if self._camera is not None:
            self._show = self._camera.show_video_feed
        return self._show
    

    @show_frames.setter
    def show_frames(self, value: bool):
        """
        Set the show status of the camera.
        Args:
            value: bool: The new show status.
        """
        self._show = value
        if self._camera is not None:
            self._camera.show_video_feed = value



    @property
    def trackers_pos(self) -> list:
        """
        Get the positions of the trackers.
        Returns:
            list: The positions of the trackers as a list of lists.
        """
        with self._lock:
            if self._tracking:
                return self._trackers_pos
            else:
                return []
            
    @property
    def trackers_leg(self) -> list:
        """
        Get the positions of the trackers.
        Returns:
            list: The positions of the trackers as a list of lists.
        """
        with self._lock:
            if self._tracking:
                return self._trackers_leg
            else:
                return []
            
    @property
    def point_cloud(self) -> np.ndarray:
        """
        Get the point cloud data.
        Returns:
            The point cloud data as a numpy array.
        """
        with self._lock:
            if self._compute_point_cloud:
                return self._point_cloud
            else:
                return np.array([])

    

    



    ##########################
    #  METHODS
    ##########################


    @staticmethod
    def listCameras() -> list:
        """
        Static method to list all the Realsense cameras connected to the computer

        Returns:
            list: A list of the serial numbers as string.
        """
        return depthcamera.listCameras()
    

    def open(self, camera_serial: str=None) -> bool:
        """
        Initialize and open the camera in another process.
        This function creates a new handle to the camera and starts it.

        Args:
            camera_serial: str: the serial number of the camera to open. If None, the first found Realsense camera will be opened. If the `camera_serial` was set as a parameter or before, the given camera will be opened.

        Returns:
            bool: True if a camera was opened, else False

        """

        try:
            if self._running:
                self.close()
                self._running = False

            if camera_serial is not None:
                self.camera_serial = camera_serial

            logger.debug("Starting camera with show: {}, tracking: {}, compute_point_cloud: {}".format(self._show, self._tracking, self._compute_point_cloud))
            self._camera = depthcamera.DepthCamera(camera_serial=self.camera_serial, 
                                compute_point_cloud=self._compute_point_cloud, 
                                show_video_feed=self._show, 
                                tracking=self._tracking)
            self.camera_serial = self._camera.camera_serial
            self._running = True
            logger.info(f"Camera {self.camera_serial} successfully started.")
            return True
        except Exception as e:
            if self._camera:
                self._camera.close()
            self._running = False
            logger.error("Error opening camera: "+str(e))
            return False

    def update(self):
        """
            Update the camera frames and tracking elements (markers and point cloud)
        """
        self._camera.update()
        with self._lock:
            # self._mask_frame = self._camera.maskFrame
            if self._tracking:
                self._trackers_pos = self._camera.trackers_pos
                self._trackers_leg = self._camera.trackers_leg
                # print("Trackers pos:", self._trackers_pos)
                # print("Trackers leg:", self._trackers_leg)

            if self._compute_point_cloud:
                    self._point_cloud = self._camera.point_cloud
        
    def close(self):
        """
        Close the camera and terminate the process. Sets the running status to False.
        """
        self._running = False
        if self._camera is not None: 
            self._camera.close()
