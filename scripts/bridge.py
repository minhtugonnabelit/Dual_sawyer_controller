import roslibpy
import threading
import time

# This script is used to open the commnuication between the two clients hosted via rosbridge server
# The protocol is as follows:
# 1. Both rosbridge server are running on two different machines (distinct IP) at background
# 2. The script will initiate the client to the each rosbridge server
# 3. Then custom topics will be created based on the desired data that need to be exchanged between the closed loop controller node running on each machine.
# 4. Data retrieved from the server will be centralized into the machine running this script and will be published to the other machines to used for multiple
#    purposes such as: collision avoidance, singularity avoidance, reachability check, etc.

class Bridge:

    """
    This class is used to open the communication between the clients hosted via rosbridge server
    The protocol is as follows:
    1. Both rosbridge server are running on two different machines (distinct IP) at background
    2. The script will initiate the client to the each rosbridge server
    3. Then custom topics will be created based on the desired data that need to be exchanged between the closed loop controller node running on each machine.
    4. Data retrieved from the server will be centralized into the machine running this script and will be published to the other machine to used for multiple
       purposes such as: collision avoidance, singularity avoidance, reachability check, etc.
    """
    def __init__(self, ip, port):
        """
        This function is used to initialize the client to the rosbridge server
        @params:
        - ip: IP address of the host machine
        - port: port number of the rosbridge server`
        """
        self._ip = ip
        self._port = port
        self._client = roslibpy.ros(host=self._ip, port=self._port)


    def topics_init(self, names):
        """
        This function is used to initialize the topics to be subscribe that will be used to exchange data between the two clients

        Possible topics to be subscribed:
        - for endpoint location feedback: /robot/limb/right/endpoint_state
        - for indicator between picking and clamping state /mission_state
        - for image subscribed /camera/rgb/image_raw
        - for tf listenner /tf

        """
        self._endpoint = roslibpy.Topic(self._client, '/robot/limb/right/endpoint_state', 'intera_core_msgs/EndpointState')
        self._mission_state = roslibpy.Topic(self._client, '/mission_state', 'std_msgs/Bool')
        self._image = roslibpy.Topic(self._client, '/camera/rgb/image_raw', 'sensor_msgs/Image')

    # TODO:
        # Handle the tf listenner that can make the communication channel to be more efficient (without the need of decompress image data retrieved from the server socker).
        # Setup the architecture to seemlessly callback data sent between controller and publish them to the other controller


    def pub_topics_init(self, names):
        """
        This function is used to initialize the topics to be publish that will be used to exchange data between the two clients

        Possible topics to be published:
        - for endpoint location feedback: /robot/limb/right/endpoint_state
        - for indicator between picking and clamping state /mission_state
        - for image subscribed /camera/rgb/image_raw
        - for tf listenner /tf

        """
        self._endpoint_pub = roslibpy.Topic(self._client, '/robot/limb/right/endpoint_state', 'intera_core_msgs/EndpointState')
        self._endpoint_sub = roslibpy.Topic(self._client, '/robot/limb/left/endpoint_state', 'intera_core_msgs/EndpointState')
        self._mission_pub = roslibpy.Topic(self._client, '/mission_state', 'std_msgs/Bool')
        self._mission_sub = roslibpy.Topic(self._client, '/mission_state', 'std_msgs/Bool')
        self._endpoint_pub.advertise()
        self._endpoint_sub.subscribe(self.endpoint_state_callback)
        self._mission_pub.advertise()
        self._mission_sub.subscribe(self.mission_state)