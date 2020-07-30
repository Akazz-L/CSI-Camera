#!/bin/bash

# Show cameras (video0 and video1) available capture width and height
# Display in stdout and Write resolutions in text file (cam1.txt and cam2.txt).
# And the available FPS

v4l2-ctl -d /dev/video0 --list-formats-ext | tee cam0.txt
v4l2-ctl -d /dev/video1 --list-formats-ext | tee cam1.txt

# Use MJPG decoder to increase fps
# https://answers.opencv.org/question/41899/changing-pixel-format-yuyv-to-mjpg-when-capturing-from-webcam/