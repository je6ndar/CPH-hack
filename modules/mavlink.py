import numpy as np

from pymavlink import mavutil
import time
import queue
from threading import Thread
import json, os

import modules.types as types
import modules.config as config


w = 640
h = 480

A = w/2
FOV = 110

flow = [0, 0]
ALTITUDE = 0

SYSTEM_ID = None
SYSTEM_COMPONENT = None
COMPONENT_ID = mavutil.mavlink.MAV_COMP_ID_USER10

MAVCONN = None
LAST_RECV_MSG_TIME = None

MAVLINK_BAUD_RATES = [921600]
MAVLINK_BAUD_RATES_worked = [] # used to save worked baud and quikly restarts if needed


MAVLINK_SERIAL = '/dev/ttyAMA0'

DESIRED_MODE = 'LOITER'

MAVLINK_SAVE_FN = 'mavlink.jsons'

ATT_FREQ_Hz = 30

MAX_TIME_WITHOUT_MSG = 2.0 # in seconds
PREV_TIME =0

MSG_TYPES = [
    'ATTITUDE',
    'UFR_HUD',
    'SERVO_OUTPUT_RAW'
]



def proc_mavlink(MavlinkSendQueue=None, SaveQueue=None):
    global MAVCONN, LAST_RECV_MSG_TIME, ALTITUDE, PREV_TIME


    if not MavlinkSendQueue:
        return

    while True:
        if not MAVCONN:
            try:
                t_conn1 = time.time()
                MAVCONN = init_mavlink()
                t_conn2 = time.time()
                print("DT mavlink-conn-init: %.3f" % (t_conn2-t_conn1))
            except RuntimeError as err:
                print("ERROR while init MAVLINK:", err)
                print("Try reinit in 1 sec")
                MAVCONN = None
                LAST_RECV_MSG_TIME = None

        if MAVCONN:
            in_msg = MAVCONN.recv_match(type=MSG_TYPES, blocking=True)#, timeout=0.05)
        else:
            in_msg = None

        if in_msg is not None:
            LAST_RECV_MSG_TIME = time.time()
            #print(in_msg)
            if in_msg.get_type() == 'UFR_HUD':
                ALTITUDE = in_msg.alt
            if in_msg.get_type() == 'ATTITUDE':
                pass

            try:
                if SaveQueue:
                    SaveQueue.put_nowait(MavlinkMessageItem(in_msg))
            except queue.Full as err:

                print("Dropped Mavlink Message", err)
        else:
            if LAST_RECV_MSG_TIME is None or time.time() - LAST_RECV_MSG_TIME > MAX_TIME_WITHOUT_MSG:
                MAVCONN = None
                LAST_RECV_MSG_TIME = None

        try:
            out_msg = MavlinkSendQueue.get_nowait()
            # get attitude from mavlink
            # if out_msg == 'ATTITUDE':
            if MAVCONN:
                MAVCONN.mav.request_data_stream_send(
                    SYSTEM_ID, SYSTEM_COMPONENT,
                    mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)

            out_msg_mavlink = convert_msg(out_msg)
            if MAVCONN and out_msg_mavlink:
                res = MAVCONN.mav.send(out_msg_mavlink)


            MavlinkSendQueue.task_done()
        except queue.Empty as err:
            pass


# wrapper class to save into file + store os_time
class MavlinkMessageItem:
    def __init__(self, msg):
        self.msg = msg
        self.os_time = time.time()

    def save(self, out_dn):
        fn = os.path.join(out_dn, MAVLINK_SAVE_FN)
        with open(fn, 'at') as out_f:
            d = self.msg.to_dict()
            d['os_time'] = self.os_time
            json.dump(d, out_f)
            out_f.write('\n')
            #out_f.flush()

def convert_msg(item):
    msg = None
    # https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
    if type(item) == types.OpticalFlow:
        flow = item.flow
        confidence = item.confidence
        msg = create_v_msg(flow, confidence, ALTITUDE)
        pass
    # https://mavlink.io/en/messages/common.html#STATUSTEXT
    elif type(item) == types.Status:
        msg = MAVCONN.mav.statustext_encode(
            severity=mavutil.mavlink.MAV_SEVERITY_INFO,
            text = item.text.encode(encoding='ascii', errors='replace')
        )

    return msg


# TODO - time?

def message_send(msg):
    if MAVCONN and msg:
                res = MAVCONN.mav.send(msg)

def create_v_msg(v,confidence, altitude):

    # Send the SET_ATTITUDE_TARGET message
    # flow_comp_m_x = (v[0]-w/2)*altitude/focal
    # flow_comp_m_y = (v[1]-h/2)*altitude/focal
    flow_comp_m_x, flow_comp_m_y, flow_x_radians, flow_y_radians = calculate_camera_motion(v[0], v[1], FOV, w, h, 100)
    print(flow_comp_m_x, flow_comp_m_y)
    if np.isnan([flow_comp_m_y, flow_comp_m_y,flow_x_radians, flow_y_radians]).any():
         return
    # Send the SET_ATTITUDE_TARGET messaged
    msg = MAVCONN.mav.optical_flow_send(
        time_usec = 0,        # Time since boot or Unix time in microseconds        
        flow_comp_m_x = flow_comp_m_x,    
        flow_comp_m_y = flow_comp_m_y,    
        ground_distance = altitude,  
        flow_x = int(flow_x_radians * 1000),#v[0],           
        flow_y = int(flow_y_radians * 1000),#v[1],
        sensor_id = 0,           
        quality = int(confidence*255),          
        flow_rate_x = 0,      
        flow_rate_y = 0
    )

    return msg




def init_mavlink():
    heartbeat_msg = None
    global MAVLINK_BAUD_RATES_worked
    for baud in MAVLINK_BAUD_RATES_worked + MAVLINK_BAUD_RATES:
        print("MAVLINK try baud=", baud)
        mavconn = mavutil.mavlink_connection(MAVLINK_SERIAL, baud=baud)
        heartbeat_msg = mavconn.wait_heartbeat(timeout=2) 
        if heartbeat_msg is not None:
            MAVLINK_BAUD_RATES_worked = [baud]
            break

        # not connected
    if not heartbeat_msg:
        return None
    
    global SYSTEM_ID, SYSTEM_COMPONENT
    SYSTEM_ID = mavconn.target_system
    SYSTEM_COMPONENT = mavconn.target_component

    print("Heartbeat from system (system %u component %u)" % (mavconn.target_system, mavconn.target_component))
    #perform_timesync_from_systime(mavconn)

    # request_message_interval(mavconn, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 100)
    # mavutil.mav.
    # request_message_interval(mavconn, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 100)
    #request_message_interval(mavconn, mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS_RAW, ATT_FREQ_Hz)#mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS_RAW)

    return mavconn





def request_message_interval(master, message_id: int, frequency_hz: float):

    """
    Request MAVLink message in a desired frequency,
    documentation for SET_MESSAGE_INTERVAL:
        https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

    Args:
        message_id (int): MAVLink message ID
        frequency_hz (float): Desired frequency in Hz
    """
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # The MAVLink message ID
        1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
        0, 0, 0, 0, # Unused parameters
        0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )



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
