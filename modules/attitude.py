# standard libraries
import cv2
import os
import shutil
import platform
import numpy as np
# import json
from time import sleep
from timeit import default_timer as timer
# from itertools import count
# from datetime import datetime

# my libraries
from video_classes import CustomVideoCapture
from crop_and_scale import get_cropping_and_scaling_parameters, crop_and_scale
from find_horizon import HorizonDetector
from global_variables import settings

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
    inference_aspect_ratio = INFERENCE_RESOLUTION[0]/INFERENCE_RESOLUTION[1]
    aspect_ratio = RESOLUTION[0]/RESOLUTION[1]
    if inference_aspect_ratio > aspect_ratio:
        print(f'The specified inference aspect ratio of {inference_aspect_ratio} is '\
                f'wider than the aspect ratio of {aspect_ratio}. This is not allowed')
        inference_height = INFERENCE_RESOLUTION[1]
        inference_width = int(np.round(INFERENCE_RESOLUTION[1] * aspect_ratio))
        INFERENCE_RESOLUTION = (inference_width, inference_height)
        print(f'The inference resolution has been adjusted to: {INFERENCE_RESOLUTION}')

    video_capture = CustomVideoCapture(RESOLUTION, SOURCE)

    video_capture.start_stream()
    sleep(1)

    crop_and_scale_parameters = get_cropping_and_scaling_parameters(video_capture.resolution, INFERENCE_RESOLUTION)
    horizon_detector = HorizonDetector(EXCLUSION_THRESH, FOV, ACCEPTABLE_VARIANCE, INFERENCE_RESOLUTION)
    frame = video_capture.read_frame()

    scaled_and_cropped_frame = crop_and_scale(frame, **crop_and_scale_parameters)

    output = horizon_detector.find_horizon(scaled_and_cropped_frame)
    roll, pitch, variance, is_good_horizon, _ = output

    return roll, pitch, variance, is_good_horizon