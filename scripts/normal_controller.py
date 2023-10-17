
import rospy
from intera_interface import Limb
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)

from collections import deque
from std_msgs.msg import (Bool, UInt16, String)
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from intera_core_msgs.msg import EndpointState
from intera_motion_msgs.msg import TrajectoryOptions
from math import pi

#################################
# EXTERNAL SIGNAL TRIGGER CLASS #
#################################

class Client_signal():

    def __init__(self):

        self.indicator = True
        rospy.Subscriber('/control_test', Bool, self.control_test_callback)

        self.text = ""
        rospy.Subscriber('/simple_action_state', String, self.action_state_callback)

        self.js = JointState()
        self.js_flag = False
        rospy.Subscriber('/action_joint_pos', JointState, self.jointstate_Callback)

        self.endpoint_state = EndpointState()   
        self.endpoint_flag = False
        rospy.Subscriber('endpoint_state', EndpointState, self.endpointstate_Callback)

        self.mission_state = rospy.Publisher('/mission_state', Bool, queue_size=10)

    ##########################
    ## Bool indicator callback
    def control_test_callback(self, msg):
        self.indicator = msg.data

    def get_indicator(self):
        return self.indicator

    #####
    ## String indicator callback
    def action_state_callback(self, msg):
        self.text = msg.data

    def get_text(self):
        return self.text

    #####
    ## joint_angle callback
    def jointstate_Callback(self, msg):
        self.js_flag = True
        self.js = msg

    def get_js(self):
        return self.js
    
    def get_js_flag(self):
        return self.js_flag
    
    #####
    ## endpoint callback
    def endpointstate_Callback(self, msg):
        self.endpoint_flag = True
        self.endpoint_state = msg

    def get_endpoint_state(self):
        return self.endpoint_state

    def get_endpoint_flag(self):
        return self.endpoint_flag
    
    #####
    ##
    def set_mission_state(self, mission_done):
        self.mission_state.publish(mission_done)
        
 
# MOTION CONTROLLER METHOD CLASS
class robot_control():

    def __init__(self, limb):

        rospy.loginfo('Robot homing!')
        self._limb = limb
        self._limb.move_to_neutral()
        rospy.loginfo('Robot homed!')

        self._tip_name = 'right_hand'
        self._ext = Client_signal()
        self._rate = rospy.Rate(5)
        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)

    def reset_state(self):
        self._reset = True

    def home_robot(self):
        if self._reset:
            self._limb.move_to_neutral()
            rospy.loginfo('Robot homed!')
            self._reset = not self._reset

    def joint_angles_control_init(self):

        try:
            self.traj = MotionTrajectory(limb = self._limb)

            wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=0.3,
                                            max_joint_accel=0.05)
            self.waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = self._limb)

            self.joint_angles = self._limb.joint_ordered_angles()

            self.waypoint.set_joint_angles(joint_angles = self.joint_angles)
            self.traj.append_waypoint(self.waypoint.to_msg())

        except rospy.ROSInterruptException:
            rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.') 

    def go_to_joint_angles(self):
        flag = True
        while not rospy.is_shutdown():

            self._pub_rate.publish(5)
            if self._ext.get_js_flag():
            # if flag:

                js = self._ext.get_js()
                desired_js = deque(js.position)
                desired_js.pop()
                desired_js.popleft()
                print(self.joint_angles)
                print(js)

                if len(desired_js) != len(self.joint_angles):
                    rospy.logerr('The number of joint_angles must be %d', len(self.joint_angles))
                    return None

                self.waypoint.set_joint_angles(joint_angles = desired_js)
                self.traj.append_waypoint(self.waypoint.to_msg())

                result = self.traj.send_trajectory()
                if result is None:
                    rospy.logerr('Trajectory FAILED to send')
                    return

                if result.result:
                    rospy.loginfo('Motion controller successfully finished the trajectory!')
                else:
                    rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                            result.errorId)
                    
                self.traj.clear_waypoints()
                flag = False

                
            else:
                rospy.loginfo("waiting for signal to execute control")
            
            self._rate.sleep()


    def cartersian_pose_init(self):
        try:
            self.traj_options = TrajectoryOptions()
            self.traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
            self.traj = MotionTrajectory(trajectory_options = self.traj_options, limb = self._limb)

            wpt_opts = MotionWaypointOptions(max_linear_speed = 0.6,
                                            max_linear_accel = 0.6,
                                            max_rotational_speed = 1.57,
                                            max_rotational_accel = 1.57,
                                            max_joint_speed_ratio = 1.0)
            self.waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = self._limb)

        except rospy.ROSInterruptException:
            rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.') 

    def go_to_Cartesian_pose(self):

        self.Done = False
        self._ext.set_mission_state(self.Done)
        # self.home_robot()

        self._pub_rate.publish(5)
        if self._ext.get_endpoint_flag():

            desired_endpoint = self._ext.get_endpoint_state()
            desired_pose = desired_endpoint.pose

            endpoint_state = self._limb.tip_state(self._tip_name)
            if endpoint_state is None:
                rospy.logerr('Endpoint state not found with tip name %s', self._tip_name)
                return None
            pose = endpoint_state.pose

            pose.position.x = desired_pose.position.x
            pose.position.y = desired_pose.position.y
            pose.position.z = desired_pose.position.z

            pose.orientation.x = desired_pose.orientation.x
            pose.orientation.y = desired_pose.orientation.y
            pose.orientation.z = desired_pose.orientation.z
            pose.orientation.w = desired_pose.orientation.w

            poseStamped = PoseStamped()
            poseStamped.pose = pose

            joint_angles = self._limb.joint_ordered_angles()
            self.waypoint.set_cartesian_pose(poseStamped, self._tip_name, joint_angles)

            # rospy.loginfo('Sending waypoint: \n%s', self.waypoint.to_string())

            self.traj.append_waypoint(self.waypoint.to_msg())

            result = self.traj.send_trajectory()
            if result is None:
                rospy.logerr('Trajectory FAILED to send')
                return

            if result.result:
                rospy.loginfo('Motion controller successfully finished the trajectory!')
            else:
                rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                            result.errorId)
            
            self.Done = result.result
            self._ext.set_mission_state(self.Done)
            
            self.traj.clear_waypoints()

        else:
            rospy.loginfo('Cartesian Pose not ready!')
        


  
def main():

    rospy.init_node('go_to_Cartesian_pose_py')
    robot = Limb()
    controller = robot_control(robot)

    controller.joint_angles_control_init()
    controller.go_to_joint_angles()




if __name__ == "__main__":
    main()