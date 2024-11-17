# mavlink SET_POSITION_TARGET_LOCAL_NED for velocity
# in MAV_FRAME_BODY_FRD coord system

class OpticalFlowVector:
    def __init__(self, v):
        self.v = v


# for text status via Mavlink STATUSTEXT msg
class Attitude:
    def __init__(self, text):
        self.text = text
