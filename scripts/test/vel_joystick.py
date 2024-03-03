import numpy as np

import rospy
from sensor_msgs.msg import JointState, Joy

import roboticstoolbox as rtb
import intera_interface
from intera_interface import CHECK_VERSION
from intera_core_msgs.msg import JointCommand


# from dual_sawyer_controller import Sawyer

POSITION_MODE = int(1)
VELOCITY_MODE = int(2)
TORQUE_MODE = int(3)
TRAJECTORY_MODE = int(4)


class Sawyer(rtb.DHRobot):

    def __init__(self) -> None:
        super().__init__(
            self._create_DH(),
            name='Sawyer',
            manufacturer='Rethink Robotics',
            keywords=('dynamics', 'symbolic'),
            )
        
        self.q = [0.0, -0.9, 0.0, 1.8, 0.0, -0.9, 0.0]

    def _create_DH(self):
        """
        Create robot's standard DH model
        """

        # deg = np.pi / 180
        mm = 1e-3
        # kinematic parameters
        a = np.r_[81, 0, 0, 0, 0, 0, 0] * mm
        d = np.r_[317, 192.5, 400, -168.5, 400, 136.3, 133.75] * mm
        alpha = [-np.pi / 2, 
                 np.pi / 2, 
                 -np.pi / 2, 
                 np.pi / 2, 
                 -np.pi / 2, 
                 np.pi / 2, 
                 0]
        qlim = np.deg2rad([[-175, 175],
                           [-219, 131],
                           [-175, 175],
                           [-175, 175],
                           [-170.5, 170.5],
                           [-170.5, 170.5],
                           [-270, 270]])

        # offset to have the dh from toolbox match with the actual pose
        offset = [0, np.pi/2, 0, 0, 0, 0, -np.pi/2]

        links = []

        for j in range(7):
            link = rtb.RevoluteDH(
                d=d[j], a=a[j], alpha=alpha[j], offset=offset[j], qlim=qlim[j])
            links.append(link)

        return links


class VelCtrl:

    _VEL_SCALE = {'linear': 0.2, 'angular': 0.3}

    def __init__(self) -> None:

        rospy.init_node("sawyer_vel_ctrl_w_joystick")
        
        self.gripper = self.gripper_ctrl_init()

        # initilize vritual sawyer model to complete jacobian calculation
        self._robot = Sawyer()

        # Initialise joystick subscriber
        self.joy_msg = None
        rospy.Subscriber("/joy", Joy, self.joy_callback)

        # Joint States Subscriber (obtain the current joint states for the vel_ctrl_sim_interface)
        rospy.Subscriber("/robot/joint_states", JointState, self.js_callback)

        # Velocity Control Message Publisher
        self.joint_command = VelCtrl.joint_command_init()
        self._joint_comm_pub = rospy.Publisher(
            "/robot/limb/right/joint_command", JointCommand, queue_size=10
        )

        # Initialise the robot head object
        self._head = intera_interface.Head()



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
                [self.joy_msg.axes[0], -self.joy_msg.axes[1], vz]) * self._VEL_SCALE['linear']
            angular_vel = np.asarray(
                [self.joy_msg.axes[3], 0, self.joy_msg.axes[4]]) * self._VEL_SCALE['angular']

            # combine velocities
            ee_vel = np.hstack((linear_vel, angular_vel))

            # calculate jacobian
            j = self._robot.jacob0(cur_js)

            # get joint velocities
            joint_vel = VelCtrl.solve_RMRC(j, ee_vel)

            # Declare the joint's limit speed (NOTE: For safety purposes, set this to a value below or equal to 0.6 rad/s. Speed range spans from 0.0-1.0 rad/s)
            limit_speed = 0.6

            # Limit joint speed to 0.6 rad/sec (NOTE: Velocity limits are surprisingly high)
            for i in range(len(joint_vel)):
                if abs(joint_vel[i]) > limit_speed:
                    joint_vel[i] = np.sign(joint_vel[i]) * limit_speed

            joint_vel = np.insert(joint_vel, 0, 0)
            joint_vel = np.insert(joint_vel, len(joint_vel), 0)

            self.joint_command.velocity = np.ndarray.tolist(joint_vel)

    # Extract the current joint states of Sawyer
    def js_callback(self, js: JointState):

        # Stores most recent joint states from the /robot/joint_states topic
        self.cur_config = js.position

    # Publishing velocity commands as a JointCommand message
    def pub_joint_ctrl_msg(self):

        # Publish the joint velocities if the grip button is pressed
        self.joint_command.header.stamp = rospy.Time.now()
        while self.joy_msg is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
            print('please press RB to start!')

        # Toggle Right Bumpper to trigger command sending
        if self.joy_msg.buttons[5]:

            # Button mapping for gripper control
            if self.joy_msg.buttons[7]: self.gripper.open()
            elif self.joy_msg.buttons[6]: self.gripper.close()
            elif self.joy_msg.buttons[8]: self.gripper.calibrate()

            self._joint_comm_pub.publish(self.joint_command)

    def run_joystick_control(self):

        while not rospy.is_shutdown():
            self.pub_joint_ctrl_msg()
            rospy.sleep(0.1)


    @staticmethod
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

    @staticmethod
    def joint_command_init():

        # Joint Command Message Initialisation
        joint_command_msg = JointCommand()

        # To check the order of the joints run a 'rostopic echo /robot/joint_states', and assign the order displayed to the JointCommand().names argument
        # This sequence is important as it is used to map the joint velocities to the correct joints, currently matched with actual Sawyer model instead of Gazebo version.
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


def main():

    try:
        # Initialise controller
        out = VelCtrl()

        # Start joystck control loop
        out.run_joystick_control()
        
        rospy.spin()

    except rospy.ROSInterruptException as e:
        raise e


if __name__ == "__main__":
    main()
