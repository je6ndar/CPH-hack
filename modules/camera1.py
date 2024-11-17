import cv2
import time
import psutil


settings = {'CAM_DEVID':1,
            'FPS_INT':30,
            'CAM_API': cv2.CAP_V4L,
            'CAM_WIDTH': 800,
            'CAM_HEIGHT': 600,
            'CAM_CODEC': 'MJPG'
            }



class WorkingFrame:
    def __init__(self, frame_idx, array, os_time=None):
        self.frame_idx  = frame_idx
        self.os_time = os_time or time.time()
        self.img = array[:, :, 2].copy()
        self.rgb = array.copy()

           
    def save(self, out_dn):
        dn = os.path.join(out_dn, 'wframes')
        if not os.path.exists(dn):
            os.mkdir(dn)

        fn = os.path.join(dn, 'w%06d.npz' % self.frame_idx)
        np.savez_compressed(fn, img=self.img)



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

def process_usbcam(RawFramesQueue=None, SaveRawFramesQueue=None):

    global settings
    print('ProcessUSBCamThread: Running')
    camera = None
    frame_idx = -1

    print("FPS_INT=", settings['FPS_INT'])
    while True:

        #if FPS_INT is None or frame_idx % FPS_INT == 0:
        #    print('PICAM: state %s' % state.STATE)
        # if state.STATE == state.IDLE:
        #     if camera:
        #         camera.close()
        #         camera, mode = None, None
        #     time.sleep(0.1)
        if True:#state.STATE >= state.CAPTURE_FRAME:
            if not camera:
                try:
                    t_cam1 = time.time()
                    camera = init_and_start_camera(settings)
                    t_cam2 = time.time()
                    print("DT cam-init: %.3f" % (t_cam2-t_cam1))
                except RuntimeError as err:
                    print("ERROR while init PICAM:", err)
                    print("Try reinit in 1 sec")
                    time.sleep(1)
                    continue

            if camera:
                t1 = time.time()
                frame_idx += 1

                ret, video_frame = camera.read()

                w_frame = WorkingFrame(frame_idx, video_frame)

                if RawFramesQueue:
                    RawFramesQueue.put(w_frame)
                    #print('raw_frame', RawFramesQueue.qsize())

                if frame_idx % settings['FPS_INT'] == 0 and SaveRawFramesQueue:
                    SaveRawFramesQueue.put(w_frame)

                t2 = time.time()
                #if frame_idx % settings['FPS_INT'] == 0:
                    #print("DT frame_idx=%s: %.3f" % (frame_idx, 1/(t2-t1)))
                    #print('cam', psutil.virtual_memory().percent)
                    #print('raw_frame_put', RawFramesQueue.qsize())



                #     camera.stop()
                #     break


