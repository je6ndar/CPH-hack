# standard libraries
import platform
import numpy as np
import time
# import json
import cv2
from time import sleep
from timeit import default_timer as timer
# from itertools import count
# from datetime import datetime

# my libraries
from modules.video_classes import CustomVideoCapture
from modules.crop_and_scale import get_cropping_and_scaling_parameters, crop_and_scale
from modules.find_horizon import HorizonDetector
from modules.global_variables import settings

ENABLE_SWITCHES = False
ENABLE_SCREEN = False

def proc_attitude():
    SOURCE = settings.get_value('source')
    RESOLUTION = settings.get_value('resolution')
    INFERENCE_RESOLUTION = settings.get_value('inference_resolution')
    FPS = settings.get_value('fps')
    ACCEPTABLE_VARIANCE = settings.get_value('acceptable_variance')
    EXCLUSION_THRESH = settings.get_value('exclusion_thresh')
    FOV = settings.get_value('fov')
    OPERATING_SYSTEM = platform.system()
    IS_STATION = OPERATING_SYSTEM == 'Linux' or OPERATING_SYSTEM == 'Darwin'

    # Validate inference_resolution
    if INFERENCE_RESOLUTION[1] > RESOLUTION[1]:
        print(f'Specified inference resolution of {INFERENCE_RESOLUTION} is '\
            f' taller than the resolution of {RESOLUTION}. This is not allowed.')
        # INFERENCE_RESOLUTION = (100, 100)
        INFERENCE_RESOLUTION = (640, 480)
        print(f'Inference resolution has been adjusted to the recommended '\
            f'resolution of {INFERENCE_RESOLUTION}.')
    print(f'Inference resolution: {INFERENCE_RESOLUTION}, {INFERENCE_RESOLUTION[0]}, {INFERENCE_RESOLUTION[1]}')
    inference_aspect_ratio = INFERENCE_RESOLUTION[0]/INFERENCE_RESOLUTION[1]
    aspect_ratio = RESOLUTION[0]/RESOLUTION[1]
    if inference_aspect_ratio > aspect_ratio:
        print(f'The specified inference aspect ratio of {inference_aspect_ratio} is '\
                f'wider than the aspect ratio of {aspect_ratio}. This is not allowed')
        inference_height = INFERENCE_RESOLUTION[1]
        inference_width = int(np.round(INFERENCE_RESOLUTION[1] * aspect_ratio))
        INFERENCE_RESOLUTION = (inference_width, inference_height)
        print(f'The inference resolution has been adjusted to: {INFERENCE_RESOLUTION}')

    sources = list_and_open_cameras()
    print(f'Sources: {sources}')
    video_capture = CustomVideoCapture(RESOLUTION, str(sources[0]))

    video_capture.start_stream()
    sleep(1)

    while True:
        crop_and_scale_parameters = get_cropping_and_scaling_parameters(video_capture.resolution, INFERENCE_RESOLUTION)
        horizon_detector = HorizonDetector(EXCLUSION_THRESH, FOV, ACCEPTABLE_VARIANCE, INFERENCE_RESOLUTION)
        frame = video_capture.read_frame()
        if frame:
            scaled_and_cropped_frame = crop_and_scale(frame, **crop_and_scale_parameters)

            output = horizon_detector.find_horizon(scaled_and_cropped_frame)
            roll, pitch, variance, is_good_horizon, _ = output
            print(f'Camera Roll: {roll}, Pitch: {pitch}, Variance: {variance}, Is good horizon: {is_good_horizon}')
            with open('camera_attitude_log.txt', 'a') as f:
                f.write(f'{time.time()} {roll} {pitch} {variance} {is_good_horizon}\n')

        # USE THESE VARIABLES IN THE REST OF THE CODE

    print('---------------------END---------------------')


def list_and_open_cameras():
    # Test camera indices to find connected cameras
    index = 0
    connected_cameras = []
    while True:
        cap = cv2.VideoCapture(index)
        if not cap.read()[0]:  # If the read fails, assume no more cameras
            break
        connected_cameras.append(index)
        cap.release()
        index += 1

    print(f"Connected cameras: {connected_cameras}")
    
    # Check if there are any cameras connected
    if not connected_cameras:
        print("No USB-connected cameras detected.")
        return
    return connected_cameras