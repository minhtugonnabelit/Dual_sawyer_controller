
# About each controller to be used for start picking the object and then clamping them into each other, the following approach can be considered:
# 1. Each will have the camera to detect the object and then pick them, then can track the stamp on objects that picked by the other robot as visual feedback (PBVS) 
#    start clamping them using the kinematics constraint from the object (stamp) pose and the end effector pose of the other robot. This approach is purely based on 
#    visual feedback and kinematics constraint which used the concept of passive visual servoing.
# 2. This approach first step of the picking pieces and then clamping them is the same as the first approach, but the second step is using the force feedback from the
#    gripper to detect the contact between the gripper and the object. This approach is a hybrid between visual feedback and force feedback (tactile).

import rospy

# Modules used specifically for the dual sawyer controller
import intera_interface
import intera_external_devices
import threading    
import time

# Consider to have PR2 modules as the setup to test on other cases, 
# especially to imply this dual-arm controller on the anthropomorphic robot

class IndivController:

    def __init__(self) -> None:
        pass

    def start(self):
        pass

    def run(self):

        rospy.on_shutdown(self.shutdown)        
        pass

    def shutdown(self):
        pass
    
    def servo_picker(self):
        pass

    def servo_clamper(self):
        pass