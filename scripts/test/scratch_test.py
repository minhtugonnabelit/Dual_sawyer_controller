
import roslibpy
# import rospy
import roboticstoolbox as rtb
# from intera_interface import Limb
import numpy as np

from math import pi
import time
import threading
import ipdb
import cv2 as cv

sawyer = rtb.models.DH.Sawyer()
# sawyer.teach(sawyer.qz)
print(sawyer)

# straight forward q
qtest = [0.0, -pi/2, 0.0, 0.0, 0.0, -0.0, 0.0]


def joint_pos_to_msg(q):
    msg = roslibpy.Message({
        'header': roslibpy.Header(1024, roslibpy.Time(0.0, 0.0), ''),
        'name': ['head_pan',
                 'right_j0',
                 'right_j1',
                 'right_j2',
                 'right_j3',
                 'right_j4',
                 'right_j5',
                 'right_j6',
                 'torso_t0'
                 ],
        'position': q,
        'velocity': [0,0,0,0,0,0,0,0,0],
        'effort': [0,0,0,0,0,0,0,0,0],

    })

    return msg


# ## Custom message maker
# def ep_message_maker(desired_pose):
#     msg = roslibpy.Message({
#         'header':   roslibpy.Header(1024, roslibpy.Time(0.0,0.0), ''),
#         'pose':     desired_pose,
#         'twist':    {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0 },
#                      'angular':{'x': 0.0, 'y': 0.0, 'z': 0.0 }},
#         'wrench':   {'force':  {'x': 0.0, 'y': 0.0, 'z': 0.0 },
#                      'torque': {'x': 0.0, 'y': 0.0, 'z': 0.0 }},
#         'valid':True
#     })
#     return
# init bridge
client = roslibpy.Ros(host='192.168.0.3', port=9090)
talkerA = roslibpy.Topic(client, '/action_joint_pos', 'sensor_msgs/JointState')

client.run()

message = joint_pos_to_msg(qtest)

talkerA.advertise()
talkerA.publish(message=message)
# print(message)
