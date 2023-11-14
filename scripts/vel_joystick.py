import numpy as np

import rospy
import tf
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool

import intera_interface
from intera_interface import CHECK_VERSION
from intera_core_msgs.msg import JointCommand

import roboticstoolbox as rtb
import spatialmath as sm
import modern_robotics as mr
import sawyer_MR_description as s_des
from scripts.robot.sawyer import Sawyer

POSITION_MODE = int(1)
VELOCITY_MODE = int(2)
TORQUE_MODE = int(3)
TRAJECTORY_MODE = int(4)

VEL_SCALE = {'linear': 0.3, 'angular': 0.8}




class VelCtrl:

    def __init__(self) -> None:

        rospy.init_node("sawyer_vel_ctrl_w_joystick")

        # Initialise joystick subscriber
        self.joy_msg = None
        rospy.Subscriber("/joy", Joy, self.joy_callback)

        # Joint States Subscriber (obtain the current joint states for the vel_ctrl_sim_interface)
        rospy.Subscriber("/robot/joint_states", JointState, self.js_callback)

        # Velocity Control Message Publisher
        self.joint_command = self.joint_command_init()
        self.pub = rospy.Publisher(
            "/robot/limb/right/joint_command", JointCommand, queue_size=10
        )

        

        # initilize vritual sawyer model to complete jacobian calculation
        self._robot = Sawyer()

        # Set initial joint states to 0, may need to change in IRL use
        self.cur_config = np.zeros(9)

        # Wait for actual joint_states to be stored by js_store() callback (NOTE: Don't change the 'is' to '==')
        while np.sum(self.cur_config) is 0:

            # If the current joint configurations of the robot are set to 0 put the thread to sleep (similar to a rate_limiter.sleep())
            rospy.sleep(0.1)


    def gripper_ctrl_init(self):
        """
        """

        # Initialise interfaces
        rs = intera_interface.RobotEnable(CHECK_VERSION)
        init_state = rs.state()
        gripper = None
        original_deadzone = None

        rp = intera_interface.RobotParams()
        valid_limbs = rp.get_limb_names()

        # Cleaning function (executed on shutdown)
        def clean_shutdown():
            if gripper and original_deadzone:
                gripper.set_dead_zone(original_deadzone)

        try:
            # Instantiate the gripper object
            gripper = intera_interface.Gripper(
                valid_limbs[0] + "_gripper")
        except (ValueError, OSError) as e:
            rospy.logerr(
                "Could not detect an electric gripper attached to the robot.")
            clean_shutdown()
            return
        rospy.on_shutdown(clean_shutdown)

        # Possible deadzone values: 0.001 - 0.002
        original_deadzone = gripper.get_dead_zone()
        gripper.set_dead_zone(0.001)

        return gripper


    def joint_command_init(self):

        # Joint Command Message Initialisation
        joint_command_msg = JointCommand()

        # To check the order of the joints run a 'rostopic echo /robot/joint_states', and assign the order displayed to the JointCommand().names argument
        joint_command_msg.names = [
            "head_pan",
            "right_j0",
            "right_j1",       
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

        self.joy_msg = msg

        while msg is None:
            rospy.INFO("Press RB to start")
            rospy.sleep(0.1)

        if len(self.cur_config) >= 7:
            cur_js = np.delete(self.cur_config, [0, -1])

            vz = 0
            if self.joy_msg.buttons[1]:
                vz = 0.5
            elif self.joy_msg.buttons[2]:
                vz = -0.5

            # get linear and angular velocity
            linear_vel = np.asarray(
                [-self.joy_msg.axes[1], self.joy_msg.axes[0], vz]) * VEL_SCALE['linear']
            angular_vel = np.asarray(
                [self.joy_msg.axes[3], self.joy_msg.axes[4], 0]) * VEL_SCALE['angular']

            # combine velocities
            ee_vel = np.hstack((linear_vel, angular_vel))

            # calculate jacobian
            j = self._robot.jacob0(cur_js)

            # get joint velocities
            joint_vel = VelCtrl.solve_RMRC(j, ee_vel)

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

        # Publish the joint velocities if the grip button is pressed
        self.joint_command.header.stamp = rospy.Time.now()
        while self.joy_msg is None:
            rospy.sleep(0.1)
            print('please press RB to start!')

        if self.joy_msg.buttons[5]:
            self.pub.publish(self.joint_command)


    def solve_RMRC(jacob, ee_vel):

        # calculate manipulability
        w = np.sqrt(np.linalg.det(jacob @ np.transpose(jacob)))

        # set threshold and damping
        w_thresh = 0.04
        max_damp = 0.5

        # if manipulability is less than threshold, add damping
        if w < w_thresh:
            damp = (1-np.power(w/w_thresh, 2)) * max_damp 
        else: 
            damp = 0

        # calculate damped least square
        j_dls = np.transpose(jacob) @ np.linalg.inv( jacob @ np.transpose(jacob) + damp * np.eye(6) )

        # get joint velocities, if robot is in singularity, use damped least square
        joint_vel = j_dls @ np.transpose(ee_vel)

        return joint_vel

    def run_joystick_control(self):

        while not rospy.is_shutdown():
            self.pub_joint_ctrl_msg()
            rospy.sleep(0.1)




def main():
    try:
        # Initialise controller
        out = VelCtrl()

        # Publish the joint velocities to get the robot running
        out.run_joystick_control()
        

        rospy.spin()
    except rospy.ROSInterruptException as e:
        raise e


if __name__ == "__main__":
    main()
