import cv2
import numpy as np

def get_cropping_and_scaling_parameters(original_resolution: tuple, new_resolution: int) -> dict:
    """
    original_resolution: resolution of the original, unscaled frame
    new_resolution: desired resolution for performing inferences, genereally much
                    smaller than original_resolution, e.g. (100, 100)
    """
    new_aspect_ratio = new_resolution[0] / new_resolution[1]
    original_aspect_ratio = original_resolution[0] / original_resolution[1]

    if new_aspect_ratio > original_aspect_ratio:
        print(f"Requested aspect ratio of {new_aspect_ratio} is wider than original aspect ratio of {original_aspect_ratio}. "\
                "This is not allowed.")
        new_aspect_ratio = original_aspect_ratio
        print(f'Aspect ratio of {new_aspect_ratio} will be used instead.')

    # define some variables related to cropping
    height = original_resolution[1]
    width = original_resolution[0]
    new_width = height * new_aspect_ratio
    margin = (width - new_width) // 2
    cropping_start = int(margin)
    cropping_end = int(width - margin)
    # define some variables related to scaling
    scale_factor = new_resolution[1] / original_resolution[1]
    # convert to dictionary
    crop_and_scale_parameters = {}
    crop_and_scale_parameters['cropping_start'] = cropping_start
    crop_and_scale_parameters['cropping_end'] = cropping_end
    crop_and_scale_parameters['scale_factor'] = scale_factor

    return crop_and_scale_parameters

def crop_and_scale(frame, cropping_start, cropping_end, scale_factor):
    # crop the image
    frame = frame[:,cropping_start:cropping_end]
    # resize the image
    frame = cv2.resize(frame, (0, 0), fx=scale_factor, fy=scale_factor)
    return frame