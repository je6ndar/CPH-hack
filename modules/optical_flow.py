import numpy as np
import cv2
import modules.mavlink as mav
import modules.types as type

w = 640
h = 480

A = w/2
FOV = 110

settings = {'CAM_DEVID':0,
            'FPS_INT':30,
            'CAM_API': cv2.CAP_V4L,
            'CAM_WIDTH': 640,
            'CAM_HEIGHT': 480,
            'CAM_CODEC': 'MJPG'
            }

def init_and_start_camera(settings):

    camera = cv2.VideoCapture(settings['CAM_DEVID'], settings['CAM_API'])
    print(settings['CAM_CODEC'])

    try:
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*settings['CAM_CODEC']))
    except:
        print(f"Unsuported camera codec {(settings['CAM_CODEC'])}")

    camera.set(cv2.CAP_PROP_FPS, settings['FPS_INT'])
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, settings['CAM_WIDTH'])
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, settings['CAM_HEIGHT'])


    return camera

def proc_optflow(MavlinkSendQueue = None):
    if not MavlinkSendQueue:
        return
    camera = init_and_start_camera(settings)
    _, prev_frame = camera.read()
    prev_frame = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
    while True:
        _, current_frame = camera.read()
        current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        flow, confidence = lightweight_optical_flow(prev_frame, current_frame)
        prev_frame = current_frame
        if flow.any() == None:
             continue    
        print(flow[0], flow[1])
        print(confidence)
        optFlow = type.OpticalFlow(flow, confidence)
        MavlinkSendQueue.put_nowait()
        #time.sleep(1/5)
        #print("desired quaternion:", get_quaternion(CURRENT_QUATERNION, v))
        #stime.sleep(2)


def calculate_camera_motion(flow_x, flow_y, fov, width, height, distance_to_surface):
    """
    Calculate the camera's motion in real-world units based on optical flow and camera parameters.

    :param flow_x: Optical flow displacement in pixels (x-direction).
    :param flow_y: Optical flow displacement in pixels (y-direction).
    :param fov_x: Camera's horizontal field of view (degrees).
    :param fov_y: Camera's vertical field of view (degrees).
    :param width: Image width in pixels.
    :param height: Image height in pixels.
    :param distance_to_surface: Distance from the camera to the surface (meters or same units as result).
    :return: Real-world displacement (Delta_X, Delta_Y, Total_Distance).
    """
    # Convert FOV to radians
    fov_x_rad = np.radians(fov)
    fov_y_rad = np.radians(fov)

    # Convert pixel displacements to angular displacements
    theta_x = flow_x * (fov_x_rad / width)  # Angular displacement in radians (x)
    theta_y = flow_y * (fov_y_rad / height)  # Angular displacement in radians (y)

    # Convert angular displacements to real-world distances
    delta_x = distance_to_surface * np.tan(theta_x)
    delta_y = distance_to_surface * np.tan(theta_y)


    return delta_x, delta_y, theta_x, theta_y

def lightweight_optical_flow(frame1, frame2):
    # Convert frames to grayscale
    # gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    # gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
    gray1 = frame1
    gray2 = frame2
    if frame1.any() == None or frame2.any() == None:
         return None
    # Detect sparse key points
    features = cv2.goodFeaturesToTrack(gray1, maxCorners=200, qualityLevel=0.3, minDistance=1)
    
    if features is None or len(features) == 0:
        print("No features detected!")
        return None, None  # Or handle it appropriately


    # Calculate optical flow for these key points
    new_features, status, _ = cv2.calcOpticalFlowPyrLK(gray1, gray2, features, None)
    
    confidence = np.mean(status)
    
    # Compute the flow vectors for tracked points
    valid_points = status == 1
    flow_vectors = new_features[valid_points] - features[valid_points]

    # Average the flow vectors to get a single directional vector
    avg_flow = np.mean(flow_vectors, axis=0)
    return avg_flow, confidence # [dx, dy]