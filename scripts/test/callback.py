import roslibpy

## Class to handle callback data
class Callback:

    _endpoint = list()
    _endpoint_ready = False
    _mission_status = False

    def __init__(self, client):

        self._client = client

        self._endpoint_sub = roslibpy.Topic(self._client,'/robot/limb/right/endpoint_state', 'intera_core_msgs/EndpointState', throttle_rate=50)
        self._endpoint_sub.subscribe(self.endpoint_state_callback)

        self.mission_subscriber = roslibpy.Topic(self._client,'/mission_state', 'std_msgs/Bool')
        self.mission_subscriber.subscribe(self.mission_state)

    ### ENdpoint state data getter
    def endpoint_state_callback(self, msg):
        self._endpoint_ready = True
        self._endpoint = msg

    def get_endpoint_state(self):
        return self._endpoint
    
    def get_endpoint_flag(self):
        return self._endpoint_ready
    
    ### Mission state getter
    def mission_state(self, msg):
        self._mission_status = msg['data']

    def get_status(self):
        return self._mission_status