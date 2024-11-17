import modules.state as state
import modules.camera0 as camera0
import modules.camera1 as camera1
import modules.mavlink as mavlink
import modules.optical_flow as optflow
import modules.attitude as attitude

import modules.config as config

import queue
from threading import Thread

import time, sys, os

def run():
    # Frames_Camera0_Queue = queue.Queue() # keep raw picam frames #RawFramesQueue
    # Frames_Camera1_Queue = queue.Queue() # keep all data to save on SD card - frames,mavlink msg, target msg
    MavlinkSendQueue = queue.Queue() # msgs to send via mavlink
    SaveQueue = queue.Queue()

    # WebShowQueue = queue.Queue() # img to visualise via web-server

    #get frames from camera0
    # ProcessCamera0Thread= Thread(target=camera0.process_usbcam, kwargs=\
    #     dict(RawFramesQueue=Frames_Camera0_Queue))
    
    #get frames from camera1
    # ProcessCamera1Thread = Thread(target=camera1.process_usbcam, kwargs=\
    #     dict(RawFramesQueue=Frames_Camera1_Queue))
    
    # main frame processing, target detection/tracking
    OpticalFlowThread = Thread(target=optflow.proc_optflow, kwargs=dict(MavlinkSendQueue = MavlinkSendQueue))
        # dict(
        #     InputFramesQueue=Frames_Camera0_Queue,
        #     MavlinkSendQueue=MavlinkSendQueue)
    # )
    AttitudeThread = Thread(target=attitude.proc_attitude, kwargs=dict())
    # save the data on SD card
    SaveThread = Thread(target=save.save_data, kwargs=dict(SaveQueue=SaveQueue))
    # recv/send data via mavlink
    MavlinkThread = Thread(target=mavlink.proc_mavlink)


    # create visual imgs
    #WebShowThread = Thread(target=web.web_show, kwargs=dict(WebShowQueue=WebShowQueue))
    # serve web pages and screaming MJPEG
    #WebServerThread = Thread(target=web.server_thread)

    # CompositeThread = Thread(target=composite.get_image, kwargs=dict(FramesQueue=WebShowQueue))

    # threads = [ProcessPicamThread, ProcessThread, MavlinkThread, SaveThread]
    threads = [OpticalFlowThread, AttitudeThread, MavlinkThread]
    # if config.USE_WEB:
    #     threads += [WebShowThread, WebServerThread]
    # else:
    #     threads += [CompositeThread]
        
    for th in threads:
         th.start()

    # for debug
    # state.next_state(state.EV_RC_MED)
    # time.sleep(5)
    # state.next_state(state.EV_RC_HIGH)


if __name__ == "__main__":
    print("PID = ", os.getpid())
    run()
    #time.sleep(100)
