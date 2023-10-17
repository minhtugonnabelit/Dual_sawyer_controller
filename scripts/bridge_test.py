
import logging
import time
import threading

from callback import Callback
import roslibpy
import numpy as np


# Configure logging
fmt = '%(asctime)s %(levelname)8s: %(message)s'
logging.basicConfig(format=fmt, level=logging.INFO)
log = logging.getLogger(__name__)

## Custom message maker
def ep_message_maker(desired_pose):
    msg = roslibpy.Message({
        'header':   roslibpy.Header(1024, roslibpy.Time(0.0,0.0), ''),  
        'pose':     desired_pose,
        'twist':    {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0 },
                     'angular':{'x': 0.0, 'y': 0.0, 'z': 0.0 }},
        'wrench':   {'force':  {'x': 0.0, 'y': 0.0, 'z': 0.0 },
                     'torque': {'x': 0.0, 'y': 0.0, 'z': 0.0 }},
        'valid':True
    })
    return msg

# Configure threading convar
ready = None
cond = threading.Condition()

# threading function 
def transmit(talker, msg):
    with cond:
        while ready is None:
            cond.wait()
    # while done:
    talker.advertise()
    talker.publish(msg)

def start_sending():
    global ready
    with cond:
        ready = input('Whenever you ready: ')
        cond.notify_all()

## Laterncy update
def latency_check(cb1 : Callback, cb2 :Callback):
    count = False
    done = False
    while not done:
        if cb1.get_status() or cb2.get_status():
            if count is False:
                start = time.perf_counter()
                count = True
        if cb1.get_status() and cb2.get_status():
            end = time.perf_counter()
            t_miss = end-start
            done = True

    return t_miss

def main():

    ## init client to the ip hosting rosbridge server
    clientA = roslibpy.Ros(host='192.168.0.3', port=9090)       # client with the enpoint to be followed
    clientB = roslibpy.Ros(host='192.168.0.5', port=8080)     # client with the endpoint to follow

    # ## open listenner to track mission state
    Call_A = Callback(client=clientA)
    Call_B = Callback(client=clientB)
    
    ## open talker with custom topic 
    talkerA = roslibpy.Topic(clientA,'/endpoint_state', 'intera_core_msgs/EndpointState')
    talkerB = roslibpy.Topic(clientB,'/endpoint_state', 'intera_core_msgs/EndpointState')

    ## start client communication
    clientA.run()
    clientB.run()

    ## sample pose to go
    desired_pose = {'position':{'x': 0.7,
                                'y': 0.3, 
                                'z': 0.5 },
                    'orientation':{'x': 0.0,
                                'y': 1.0,
                                'z': 0.0,
                                'w': 0.0 }}
    msg5 = ep_message_maker(desired_pose=desired_pose)

    # threaded transmit
    t1 = threading.Thread(target=transmit, args=(talkerA, msg5), name='robot_A')
    t2 = threading.Thread(target=transmit, args=(talkerB, msg5), name='robot_B')
    openner = threading.Thread(target=start_sending)
    
    t1.start()
    t2.start()

    global ready
    with cond:
        ready = input('Whenever you ready: ')
        cond.notify_all()

    latency = latency_check(Call_A, Call_B)
    print('Latency: ', latency)

    t1.join()
    t2.join()

if __name__ == '__main__':
    main()





