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
from video_classes import CustomVideoWriter, CustomVideoCapture
import global_variables as gv
from crop_and_scale import get_cropping_and_scaling_parameters, crop_and_scale
from find_horizon import HorizonDetector
from draw_display import draw_horizon, draw_hud, draw_roi
from disable_wifi_and_bluetooth import disable_wifi_and_bluetooth
from flight_controller import FlightController
from global_variables import settings

ENABLE_SWITCHES = False
ENABLE_SCREEN = False

def attitude():
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

    # global variables
    actual_fps = 0
    if IS_STATION:
        # For performance reasons, default to not rending the HUD when
        # running on Raspberry Pi.
        render_image = False
    else:
        render_image = True

    # functions
    def finish_recording():
        """
        Finishes up the recording and saves the diagnostic data file.
        """
        datadict = {}
        metadata = {}
        frames = {}
        datadict['metadata'] = metadata
        datadict['frames'] = frames
        # metadata['datetime'] = dt_string
        metadata['ail_kp'] = settings.get_value('ail_kp')
        metadata['elev_kp'] = settings.get_value('elev_kp')
        metadata['max_deflection'] = settings.get_value('max_deflection')
        metadata['servos_reversed'] = settings.get_value('servos_reversed')
        metadata['fps'] = FPS
        metadata['inference_resolution'] = INFERENCE_RESOLUTION
        metadata['resolution'] = RESOLUTION
        metadata['acceptable_variance'] = ACCEPTABLE_VARIANCE
        metadata['exclusion_thresh'] = EXCLUSION_THRESH
        metadata['fov'] = FOV

        # wait for video_writer to finish recording      
        # while video_writer.run:
        #     sleep(.01) 

        print('Saving diagnostic data...')
        # with open(f'{file_path}/{dt_string}.json', 'w') as convert_file: 
        #     convert_file.write(json.dumps(datadict))
        print('Diagnostic data saved.')
            
        thumbdrive = '/media/pi/scratch'
        if not os.path.exists(thumbdrive):
            return
        
        dst = f'{thumbdrive}/recordings'
        if not os.path.exists(dst):
            os.makedirs(dst)
        
        src_folder = '/home/pi/horizon_detector/recordings'
        for file in os.listdir(src_folder):
                shutil.copy(f'{src_folder}/{file}', dst)
        
    # paused_frame = np.zeros((500, 500, 1), dtype = "uint8")
    # cv2.putText(paused_frame, 'Real-time display is paused.',(20,30),cv2.FONT_HERSHEY_COMPLEX_SMALL,.75,(255,255,255),1,cv2.LINE_AA)
    # cv2.putText(paused_frame, "Press 'd' to enable real-time display.",(20,60),cv2.FONT_HERSHEY_COMPLEX_SMALL,.75,(255,255,255),1,cv2.LINE_AA)
    # cv2.putText(paused_frame, "Press 'r' to record.",(20,90),cv2.FONT_HERSHEY_COMPLEX_SMALL,.75,(255,255,255),1,cv2.LINE_AA)
    # cv2.putText(paused_frame, "Press 'q' to quit.",(20,120),cv2.FONT_HERSHEY_COMPLEX_SMALL,.75,(255,255,255),1,cv2.LINE_AA)
    # cv2.imshow("Real-time Display", paused_frame)

    # # define VideoCapture
    video_capture = CustomVideoCapture(RESOLUTION, SOURCE)

    # start VideoStreamer
    video_capture.start_stream()
    sleep(1)

    # # get some parameters for cropping and scaling
    crop_and_scale_parameters = get_cropping_and_scaling_parameters(video_capture.resolution, INFERENCE_RESOLUTION)

    # read the image
    # source_img_path = '../../Downloads/test4.png'
    # frame = cv2.imread(source_img_path)
    # if frame is None:
    #     print(f'Failed to read image from {SOURCE}. Terminating program.')
    #     return

    # get some parameters for cropping and scaling
    # crop_and_scale_parameters = get_cropping_and_scaling_parameters(frame.shape[:2], INFERENCE_RESOLUTION)
    
    # define the HorizonDetector
    horizon_detector = HorizonDetector(EXCLUSION_THRESH, FOV, ACCEPTABLE_VARIANCE, INFERENCE_RESOLUTION)
    
    # initialize some values related to the flight controller
    # recording_switch_new_position = None
    # autopilot_switch_new_position = None
    ail_stick_val, elev_stick_val, ail_val, elev_val, flt_mode, pitch_trim, ail_trim, elev_trim = (0 for _ in range(8))
    
    
    # if ENABLE_SWITCHES and IS_STATION:
    #     from switches_and_servos import ServoHandler, TransmitterSwitch, TrimReader
        
    #     wifi_response, bluetooth_response = disable_wifi_and_bluetooth()
    #     print(f'{wifi_response} {bluetooth_response}')
        
    #     recording_switch = TransmitterSwitch(26, 2)
    #     autopilot_switch = TransmitterSwitch(6, 2)
            
    #     ail_handler = ServoHandler(13, 12, FPS, 990, 2013)
    #     elev_handler = ServoHandler(18, 27, FPS, 990, 2013)
        
    #     flt_ctrl = FlightController(ail_handler, elev_handler, FPS)
        
    #     pitch_trim_reader = TrimReader(25)
        
    t1 = timer()
    n = 0
    while True:
        frame = video_capture.read_frame()

        scaled_and_cropped_frame = crop_and_scale(frame, **crop_and_scale_parameters)

        output = horizon_detector.find_horizon(scaled_and_cropped_frame, diagnostic_mode=render_image)
        roll, pitch, variance, is_good_horizon, _ = output
            
        # if ENABLE_SWITCHES and IS_STATION:
        #     if pitch is not None:
        #         adjusted_pitch = pitch + pitch_trim
        #     else:
        #         adjusted_pitch = None
        #     ail_stick_val, elev_stick_val, ail_val, elev_val, ail_trim, elev_trim = flt_ctrl.run(roll, adjusted_pitch, is_good_horizon)
        #     flt_mode = flt_ctrl.program_id  

        # if gv.recording:
        #     recording_frame_num = next(recording_frame_iter)

        #     frame_data = {}
        #     frame_data['roll'] = roll
        #     frame_data['pitch'] = pitch
        #     frame_data['variance'] = variance
        #     frame_data['is_good_horizon'] = is_good_horizon
        #     frame_data['actual_fps'] = actual_fps
        #     frame_data['ail_val'] = ail_val
        #     frame_data['elev_val'] = elev_val
        #     frame_data['ail_stick_val'] = ail_stick_val
        #     frame_data['elev_stick_val'] = elev_stick_val
        #     frame_data['ail_trim'] = ail_trim
        #     frame_data['elev_trim'] = elev_trim
        #     frame_data['flt_mode'] = flt_mode
        #     frame_data['pitch_trim'] = pitch_trim 
        #     frames[recording_frame_num] = frame_data
         
        if render_image:
            frame_copy = frame.copy()
            draw_roi(frame_copy, crop_and_scale_parameters)
            
            if roll and is_good_horizon:
                color = (240,240,240)
                adjusted_pitch = pitch + pitch_trim
                draw_horizon(frame_copy, roll, adjusted_pitch, FOV, color, draw_groundline=False)
            
            if roll:
                if is_good_horizon:
                    color = (255,0,0)
                else:
                    color = (0,0,255)
                draw_horizon(frame_copy, roll, pitch, 
                            FOV, color, draw_groundline=is_good_horizon)

            draw_hud(frame_copy, roll, pitch, actual_fps, is_good_horizon, gv.recording)

            center = (frame_copy.shape[1]//2, frame_copy.shape[0]//2)
            radius = frame.shape[0]//100
            cv2.circle(frame_copy, center, radius, (255,0,0), 2)

            cv2.imshow("Real-time Display", frame_copy)

        # if gv.recording:
        #     video_writer.queue.put(frame)     

        # if ENABLE_SWITCHES and IS_STATION:
        #     autopilot_switch_new_position = autopilot_switch.detect_position_change()
            
        # if ENABLE_SWITCHES and IS_STATION and flt_mode != 2:
        #     recording_switch_new_position = recording_switch.detect_position_change()
        # elif ENABLE_SWITCHES and IS_STATION and flt_mode == 2:
        #     recording_switch_new_position = None
        #     pitch_trim = pitch_trim_reader.read()
               
        # key = cv2.waitKey(1)
        
        # if key == ord('q'):
        #     break
        # elif key == ord('d'):
        #     cv2.destroyAllWindows()
        #     cv2.imshow("Real-time Display", paused_frame)
        #     render_image = not render_image
        #     print(f'Real-time display: {render_image}')
        # elif autopilot_switch_new_position == 1 and flt_ctrl.program_id != 2:
        #     flt_ctrl.select_program(2)
        # elif autopilot_switch_new_position == 0 and flt_ctrl.program_id == 2:
        #     flt_ctrl.select_program(0)
        # elif (key == ord('r') or recording_switch_new_position == 1) and not gv.recording:
        #     gv.recording = not gv.recording
            
        #     recording_frame_iter = count()

        #     now = datetime.now()
        #     dt_string = now.strftime("%m.%d.%Y.%H.%M.%S")
        #     filename = f'{dt_string}.avi'

        #     file_path = 'recordings'
        #     video_writer = CustomVideoWriter(filename, file_path, video_capture.resolution, FPS)
        #     # video_writer = CustomVideoWriter(filename, file_path, RESOLUTION, FPS)
        #     video_writer.start_writing()

        #     datadict = {}
        #     metadata = {}
        #     frames = {}
        #     datadict['metadata'] = metadata
        #     datadict['frames'] = frames
            
        #     if IS_STATION:
        #         flt_ctrl.select_program(1)
                
        # elif (key == ord('r') or recording_switch_new_position == 0) and gv.recording:
        #     gv.recording = not gv.recording
            
        #     finish_recording()
            
        #     if IS_STATION:
        #         flt_ctrl.select_program(3)

        # cv2.destroyAllWindows()
        # cv2.imshow("Real-time Display", paused_frame)
        # render_image = not render_image
        # print(f'Real-time display: {render_image}')

        t2 = timer()
        waited_so_far = t2 - t1
        extra = .0000058 * FPS
        addl_time_to_wait = 1/FPS - waited_so_far - extra
        if addl_time_to_wait > 0:
            sleep(addl_time_to_wait)

        t_final = timer()
        actual_fps = 1/(t_final - t1)
        t1 = timer()
        
        n += 1
    
    # if gv.recording:
    #     gv.recording = not gv.recording
    #     finish_recording()
    # video_capture.release()
    cv2.destroyAllWindows()
    # gv.recording = False
    # gv.run = False
    print('---------------------END---------------------')