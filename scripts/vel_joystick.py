import numpy as np

import rospy
import tf
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool

import intera_interface
from intera_core_msgs.msg import JointCommand

import roboticstoolbox as rtb
import spatialmath as sm

POSITION_MODE = int(1)
VELOCITY_MODE = int(2)
TORQUE_MODE = int(3)
TRAJECTORY_MODE = int(4)

VEL_SCALE = {'linear': 0.3, 'angular': 0.8}


class VelCtrl:

    def __init__(self) -> None:

        rospy.init_node("sawyer_vel_ctrl_w_joystick")

        # Initialise joystick subscriber
        rospy.Subscriber("/joy", Joy, self.joy_callback)

        # Joint States Subscriber (obtain the current joint states for the vel_ctrl_sim_interface)
        rospy.Subscriber("/robot/joint_states", JointState, self.js_callback)

        # Velocity Control Message Publisher
        self.joint_command = self.joint_command_init()
        self.pub = rospy.Publisher(
            "/robot/limb/right/joint_command", JointCommand, queue_size=10
        )

        # initilize vritual sawyer model to complete jacobian calculation
        self._robot = rtb.models.DH.Sawyer()

        # Set initial joint states to 0, may need to change in IRL use
        self.cur_config = np.zeros(9)

        # Wait for actual joint_states to be stored by js_store() callback (NOTE: Don't change the 'is' to '==')
        while np.sum(self.cur_config) is 0:
            # If the current joint configurations of the robot are set to 0 put the thread to sleep (similar to a rate_limiter.sleep())
            rospy.sleep(0.1)


    def joint_command_init(self):

        # Joint Command Message Initialisation
        joint_command_msg = JointCommand()

        # To check the order of the joints run a 'rostopic echo /robot/joint_states', and assign the order displayed to the JointCommand().names argument
        joint_command_msg.names = [
            "head_pan",
            "right_j0",
            "right_j1",        # if self.right_grip == True:
            "right_j2",
            "right_j3",
            "right_j4",
            "right_j5",
            "right_j6",
            "torso_t0",
        ]

        # Set the control mode to the VELOCITY
        joint_command_msg.mode = VELOCITY_MODE

        # Set the initial velocity at all joints to 0.0
        joint_command_msg.velocity = np.ndarray.tolist(
            np.zeros(len(joint_command_msg.names))
        )

        return joint_command_msg

    def joy_callback(self, msg: Joy):

        # callback joystick
        cur_js = np.delete(self.cur_config, [0, -1])

        vz = 0
        if msg.buttons[1]:
            vz = 0.5
        elif msg.buttons[2]:
            vz = -0.5

        # get linear and angular velocity
        linear_vel = np.asarray(
            [msg.axes[1], - msg.axes[0], vz]) * VEL_SCALE['linear']
        angular_vel = np.asarray(
            [msg.axes[3], msg.axes[4], 0]) * VEL_SCALE['angular']

        # combine velocities
        ee_vel = np.hstack((linear_vel, angular_vel))

        # calculate jacobian
        j = self._robot.jacob0(cur_js)

        # calculate manipulability
        w = np.sqrt(np.linalg.det(j @ np.transpose(j)))

        # set threshold and damping
        w_thresh = 0.04
        max_damp = 0.5

        # if manipulability is less than threshold, add damping
        if w < w_thresh:
            damp = (1-np.power(w/w_thresh, 2)) * max_damp 
        else: 
            damp = 0

        # calculate damped least square
        j_dls = j @ np.transpose(j) @ np.linalg.inv( j @ np.transpose(j) + damp * np.eye(6) )

        # get joint velocities, if robot is in singularity, use damped least square
        joint_vel = np.linalg.pinv(j) @ j_dls @ np.transpose(ee_vel)

        # Declare the joint's limit speed (NOTE: For safety purposes, set this to a value below or equal to 0.6 rad/s. Speed range spans from 0.0-1.0 rad/s)
        limit_speed = 0.4

        # Limit joint speed to 0.4 rad/sec (NOTE: Velocity limits are surprisingly high)
        for i in range(len(joint_vel)):
            if abs(joint_vel[i]) > limit_speed:
                joint_vel[i] = np.sign(joint_vel[i]) * limit_speed
        rospy.loginfo("Joint vel: %s", joint_vel)

        joint_vel = np.insert(joint_vel, 0, 0)
        joint_vel = np.insert(joint_vel, len(joint_vel), 0)

        self.joint_command.velocity = np.ndarray.tolist(joint_vel)
        self.pub_joint_ctrl_msg()

    # Extract the current joint states of Sawyer
    def js_callback(self, js: JointState):
        """
        JointState message definition:
        std_msgs/Header header
            uint32 seq
            time stamp
            string frame_id
        string[] name
        float64[] position
        float64[] velocity
        float64[] effort
        """
        # Stores most recent joint states from the /robot/joint_states topic
        self.cur_config = js.position

    # Publishing velocity commands as a JointCommand message
    def pub_joint_ctrl_msg(self):

        # Add a header time stamp
        self.joint_command.header.stamp = rospy.Time.now()

        # Publish the joint velocities if the grip button is pressed
        self.pub.publish(self.joint_command)


def main():
    try:
        # Initialise controller
        out = VelCtrl()

        # Publish the joint velocities to get the robot running
        out.pub_joint_ctrl_msg()

        rospy.spin()
    except rospy.ROSInterruptException:
        raise e


if __name__ == "__main__":
    main()
