# MIT License
# Copyright (c) 2019,2020 JetsonHacks
# See license
# A very simple code snippet
# Using two  CSI cameras (such as the Raspberry Pi Version 2) connected to a
# NVIDIA Jetson Nano Developer Kit (Rev B01) using OpenCV
# Drivers for the camera and OpenCV are included in the base image in JetPack 4.3+

# This script will open a window and place the camera stream from each camera in a window
# arranged horizontally.
# The camera streams are each read in their own thread, as when done sequentially there
# is a noticeable lag
# For better performance, the next step would be to experiment with having the window display
# in a separate thread

import cv2
import threading
import numpy as np
import argparse
from enum import Enum

# gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
# Flip the image by setting the flip_method (most common values: 0 and 2)
# display_width and display_height determine the size of each camera pane in the window on the screen

left_camera = None
right_camera = None

# Interface Enum class
class Interface(Enum):
    USB = 1
    MIPI = 2 # CSI



# TODO CSI_Camera class to Camera class
#      adapt with interface attribute : self.interface = Interface.USB or Interface.MIPI
class CSI_Camera:

    def __init__ (self) :
        # Initialize instance variables
        # OpenCV video capture element
        self.video_capture = None
        # The last captured image from the camera
        self.frame = None
        self.grabbed = False
        # The thread where the video capture runs
        self.read_thread = None
        self.read_lock = threading.Lock()
        self.running = False

    # USB Video Capture
    def open(self, interface,sensor_id, capture_width, capture_height):
        if interface == Interface.USB:
            try:
                self.video_capture = cv2.VideoCapture(sensor_id)
                #https://docs.opencv.org/3.4/d4/d15/group__videoio__flags__base.html#gaeb8dd9c89c10a5c63c139bf7c4f5704d
                if (capture_width is not None and capture_height is not None):
                    self.video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(capture_width)) # Set width of the frame in the video frame
                    self.video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(capture_height))
                    print("Capture width and height set to : {}x{}".format(capture_width,capture_height))
                    self.video_capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
                    print("Video decoder set to : MJPG")

            except RuntimeError:
                self.video_capture = None
                print("Unable to open camera")
                print("Pipeline: " + sensor_id)
                return
            
            
        elif interface == Interface.MIPI:
            try:
                gstreamer_pipeline_string = gstreamer_pipeline(
                                                        sensor_id=sensor_id,
                                                        sensor_mode=3,
                                                        flip_method=0,
                                                        display_height=540,
                                                        display_width=960,
                                                            )
                self.video_capture = cv2.VideoCapture(
                    gstreamer_pipeline_string, cv2.CAP_GSTREAMER
                )
            
            except RuntimeError:
                self.video_capture = None
                print("Unable to open camera")
                print("Pipeline: " + gstreamer_pipeline_string)
                return

        else:
            print("Invalid interface. Only USB and MIPI interfaces are currently available")
            return

        # Display video capture info
        self.capture_info()
        # Grab the first frame to start the video capturing
        self.grabbed, self.frame = self.video_capture.read()

    def capture_info(self):
        print("Capture width and height : {}x{}".format(video_capture.get(cv2.CAP_PROP_FRAME_WIDTH),
                                                        video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        print("Capture FPS : {}".format(video_capture.get(cv2.CAP_PROP_FPS)))


    def start(self):
        if self.running:
            print('Video capturing is already running')
            return None
        # create a thread to read the camera image
        if self.video_capture != None:
            self.running=True
            self.read_thread = threading.Thread(target=self.updateCamera)
            self.read_thread.start()
        return self

    def stop(self):
        self.running=False
        self.read_thread.join()

    def updateCamera(self):
        # This is the thread to read images from the camera
        while self.running:
            try:
                grabbed, frame = self.video_capture.read()
                with self.read_lock:
                    self.grabbed=grabbed
                    self.frame=frame
            except RuntimeError:
                print("Could not read image from camera")
        # FIX ME - stop and cleanup thread
        # Something bad happened
        

    def read(self):
        with self.read_lock:
            frame = self.frame.copy()
            grabbed=self.grabbed
        return grabbed, frame

    def release(self):
        if self.video_capture != None:
            self.video_capture.release()
            self.video_capture = None
        # Now kill the thread
        if self.read_thread != None:
            self.read_thread.join()


# Currently there are setting frame rate on CSI Camera on Nano through gstreamer
# Here we directly select sensor_mode 3 (1280x720, 59.9999 fps)
def gstreamer_pipeline(
    sensor_id=0,
    sensor_mode=3,
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d sensor-mode=%d ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            sensor_mode,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

# Decode interface 
def decode(interface):
    if interface == "usb":
        return Interface.USB
    elif interface == "mipi":
        return Interface.MIPI
    else:
        print("Invalid interface. Only USB and MIPI interfaces are currently available")
        return

def start_cameras(interface, capture_width, capture_height):
    left_camera = CSI_Camera()
    left_camera.open(decode(interface),0, capture_width, capture_height)
    left_camera.start()

    right_camera = CSI_Camera()
    right_camera.open(decode(interface),1,capture_width, capture_height)
    right_camera.start()

    cv2.namedWindow("CSI Cameras", cv2.WINDOW_AUTOSIZE)

    if (
        not left_camera.video_capture.isOpened()
        or not right_camera.video_capture.isOpened()
    ):
        # Cameras did not open, or no camera attached

        print("Unable to open any cameras")
        # TODO: Proper Cleanup
        SystemExit(0)

    while cv2.getWindowProperty("CSI Cameras", 0) >= 0 :
        
        _ , left_image=left_camera.read()
        _ , right_image=right_camera.read()
        camera_images = np.hstack((left_image, right_image))
        cv2.imshow("CSI Cameras", camera_images)

        # This also acts as
        keyCode = cv2.waitKey(30) & 0xFF
        # Stop the program on the ESC key
        if keyCode == 27:
            break

    left_camera.stop()
    left_camera.release()
    right_camera.stop()
    right_camera.release()
    cv2.destroyAllWindows()





def readArgs():
    parser = argparse.ArgumentParser()
    parser.add_argument('--interface', type=str, default='usb', help='Cameras interface support (usb|mipi)')
    parser.add_argument('--capture_width', type=int, help='Cameras capture width')
    parser.add_argument('--capture_height', type=int, help='Cameras capture height')
    args = parser.parse_args()
    return args



if __name__ == "__main__":
    args = readArgs()
    start_cameras(args.interface, args.capture_width, args.capture_height)
