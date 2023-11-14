
import rospy
from std_msgs.msg import UInt16
from sensor_msgs.msg import JointState

import intera_interface
from intera_interface import Limb, CHECK_VERSION
from intera_core_msgs.msg import JointCommand
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)

import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np

from sawyer import Sawyer

POSITION_MODE = int(1)
VELOCITY_MODE = int(2)
TORQUE_MODE = int(3)
TRAJECTORY_MODE = int(4)

VEL_SCALE = {'linear': 0.3, 'angular': 0.8}
PICK_JS = [-1.507130859375, -0.065736328125, -0.6643173828125,
           1.20662109375, -0.0244140625, -1.1122587890625, 0.883083984375]
WAIT_JS = [-1.7902197265625, -0.2913447265625, -0.2379609375,
           1.800587890625, -0.2802314453125, -1.6700625, 0.379505859375]
HANG_TO_BEND = [0.00, -0.82, 0.00, 2.02, 0.00, -1.22,  0]
HANG_TO_RETURN = [-1.686873046875, -0.4110595703125, -0.2816884765625, 1.430654296875, -0.27078125, -1.1363759765625, 0.5559462890625]

# HANG_TO_BEND_POSE = #sm.SE3(0.79, 0.16, 0.24) @ sm.SE3.Ry(np.pi/2)
PICKER_GRIP_POSE = sm.SE3(0.105, 0, 0) @ sm.SE3.RPY(0, -
                                                  90, -180, unit='deg', order='xyz')
BENDER_GRIP_POSE = sm.SE3(-0.11, 0, 0) @ sm.SE3.RPY(-90,
                                                    90, -90, unit='deg', order='xyz')

#################################
# EXTERNAL SIGNAL TRIGGER CLASS #
#################################

class robot_control():

    def __init__(self, limb):

        # initilize vritual sawyer model to complete jacobian calculation
        self._robot = Sawyer()

        # Robot params setup
        self._limb = limb
        self._tip_name = 'right_hand'
        self._rate = rospy.Rate(50)

        self.gripper = self.gripper_ctrl_init()

        # Joint States Subscriber (obtain the current joint states for the vel_ctrl_sim_interface)
        self.cur_ee_pose = None
        rospy.Subscriber("/robot/joint_states", JointState, self.js_callback)

        # Velocity Control Message Publisher
        self.pub = rospy.Publisher(
            "/robot/limb/right/joint_command", JointCommand, queue_size=10)

        # Publish rate for the joint command
        self._pub_rate = rospy.Publisher(
            'robot/joint_state_publish_rate', UInt16, queue_size=10)

        self.joint_angles_control_init()

        # Set initial joint states to 0, may need to change in IRL use
        self.cur_config = np.zeros(9)

        # Wait for actual joint_states to be stored by js_store() callback (NOTE: Don't change the 'is' to '==')
        while np.sum(self.cur_config) is 0:

            # If the current joint configurations of the robot are set to 0 put the thread to sleep (similar to a rate_limiter.sleep())
            rospy.sleep(0.1)


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
        if len(self.cur_config) >= 7:

            # get current joint state from robot, remove the first and last element (head_pan and torso_t0)
            cur_js = np.delete(self.cur_config, [0, -1])
            self._robot.q = np.array(cur_js)
            self.cur_ee_pose = self._robot.fkine(self._robot.q)


    def joint_command_init(self):
        """
        """

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
    

    def joint_angles_control_init(self):
        """
        """
        try:
            self.traj = MotionTrajectory(limb=self._limb)

            wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=0.2,
                                             max_joint_accel=0.04)
            self.waypoint = MotionWaypoint(
                options=wpt_opts.to_msg(), limb=self._limb)

            self.joint_angles = self._limb.joint_ordered_angles()

            self.waypoint.set_joint_angles(joint_angles=self.joint_angles)
            self.traj.append_waypoint(self.waypoint.to_msg())

        except rospy.ROSInterruptException:
            rospy.logerr(
                'Keyboard interrupt detected from the user. Exiting before trajectory completion.')


    def home_robot(self):
        """
        """
        self.go_to_joint_angles(WAIT_JS)
        self._robot.q = np.array(WAIT_JS)


    def get_ee_pose(self):
        """
        """
        while self.cur_ee_pose is None:
            rospy.sleep(0.1)

        return self.cur_ee_pose
        # return self._robot.fkine(self._robot.q)


    def solve_RMRC(jacob, ee_vel):
        """
        """
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
        j_dls = np.transpose(jacob) @ np.linalg.inv(jacob @
                                                    np.transpose(jacob) + damp * np.eye(6))

        # get joint velocities, if robot is in singularity, use damped least square
        joint_vel = j_dls @ np.transpose(ee_vel)

        return joint_vel


    def single_step_control(self, pose, time_step, tolerance=0.001):
        """
        Moves the robot end-effector to a desired pose in Cartesian space using a single step.

        Args:
            pose (sm.SE3): The desired pose of the end-effector.
            time_step (float): The time step for the motion.
            tolerance (float, optional): The tolerance for the motion. Defaults to 0.001.

        Returns:
            None
        """

        # self._robot_busy = True

        prev_ee_pos = self._robot.fkine(self._robot.q).A[0:3, 3]
        desired_ee_pos = pose.A[0:3, 3]

        # get linear velocity between interpolated point and current pose of ee

        lin_vel = (desired_ee_pos - prev_ee_pos) / time_step

        # get angular velocity between interpolated ...

        s_omega = (pose.A[0:3, 0:3] @ np.transpose(self._robot.fkine(
            self._robot.q).A[0:3, 0:3]) - np.eye(3)) / time_step
        ang_vel = [s_omega[2, 1], s_omega[0, 2], s_omega[1, 0]]

        # combine velocities

        ee_vel = np.hstack((lin_vel, ang_vel))

        # calculate joint velocities, singularity check is already included in the function

        j = self._robot.jacob0(self._robot.q)
        mu_threshold = 0.04 if self._robot._name == "Sawyer" else 0.01
        joint_vel = robot_control.solve_RMRC(j, ee_vel)

        return joint_vel


    def follow_cart_path(self, path, speed=1):
        """
        """
        index = 0

        # if isinstance(path, list):
        while index < len(path):

            if len(self.cur_config) >= 7:

                # get current joint state from subscribing limb jointstates, remove the first and last element (head_pan and torso_t0)
                cur_js = np.delete(self.cur_config, [0, -1])
                self._robot.q = np.array(cur_js)

                # get joint velocities
                joint_vel = self.single_step_control(path[index], 0.2)

                # Declare the joint's limit speed (NOTE: For safety purposes, set this to a value below or equal to 0.6 rad/s. Speed range spans from 0.0-1.0 rad/s)
                limit_speed = 0.4

                # Limit joint speed to 0.4 rad/sec (NOTE: Velocity limits are surprisingly high)
                for i in range(len(joint_vel)):
                    if abs(joint_vel[i]) > limit_speed:
                        joint_vel[i] = np.sign(joint_vel[i]) * limit_speed

                self._robot.q = self._robot.q + joint_vel * 0.2

                # format the joint velocity to be published
                joint_vel = np.insert(joint_vel, 0, 0)
                joint_vel = np.insert(joint_vel, len(joint_vel), 0)

                joint_command = self.joint_command_init()
                joint_command.velocity = np.ndarray.tolist(joint_vel)
                joint_command.header.stamp = rospy.Time.now()
                self.pub_joint_ctrl_msg(joint_command)
                index += 1
                self._rate.sleep()

            else:
                continue

        # # if isinstance(path, list):
        # while index < len(path):

        #     # get joint velocities
        #     joint_vel = self.single_step_control(path[index], 0.2)

        #     # Declare the joint's limit speed (NOTE: For safety purposes, set this to a value below or equal to 0.6 rad/s. Speed range spans from 0.0-1.0 rad/s)
        #     limit_speed = 0.4

        #     # Limit joint speed to 0.4 rad/sec (NOTE: Velocity limits are surprisingly high)
        #     for i in range(len(joint_vel)):
        #         if abs(joint_vel[i]) > limit_speed:
        #             joint_vel[i] = np.sign(joint_vel[i]) * limit_speed
        #     rospy.loginfo("Joint vel: %s", joint_vel)

        #     # set virtual joint states
        #     self._robot.q = self._robot.q + joint_vel * 0.2

        #     # format the joint velocity to be published
        #     joint_vel = np.insert(joint_vel, 0, 0)
        #     joint_vel = np.insert(joint_vel, len(joint_vel), 0)
        #     joint_command = self.joint_command_init()
        #     joint_command.velocity = np.ndarray.tolist(joint_vel)
        #     joint_command.header.stamp = rospy.Time.now()

        #     # publish joint velocity
        #     self.pub_joint_ctrl_msg(joint_command)
        #     index += 1
        #     self._rate.sleep()


    def go_to_joint_angles(self, desired_js):

        if len(desired_js) != len(self.joint_angles):
            rospy.logerr('The number of joint_angles must be %d',
                         len(self.joint_angles))
            return None

        self.waypoint.set_joint_angles(joint_angles=desired_js)
        self.traj.append_waypoint(self.waypoint.to_msg())

        result = self.traj.send_trajectory()
        if result is None:
            rospy.logerr('Trajectory FAILED to send')
            return

        if result.result:
            rospy.loginfo(
                'Motion controller successfully finished the trajectory!')
        else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)

        self.traj.clear_waypoints()


    def pub_joint_ctrl_msg(self, msg:  JointCommand):

        # Publish the joint velocities if the grip button is pressed
        msg.header.stamp = rospy.Time.now()
        self.pub.publish(msg)


    def send_vel_command(self, joint_vel):
        # Declare the joint's limit speed (NOTE: For safety purposes, set this to a value below or equal to 0.6 rad/s. Speed range spans from 0.0-1.0 rad/s)
        limit_speed = 0.4

        # Limit joint speed to 0.4 rad/sec (NOTE: Velocity limits are surprisingly high)
        for i in range(len(joint_vel)):
            if abs(joint_vel[i]) > limit_speed:
                joint_vel[i] = np.sign(joint_vel[i]) * limit_speed

        self._robot.q = self._robot.q + joint_vel * 0.2

        # format the joint velocity to be published
        joint_vel = np.insert(joint_vel, 0, 0)
        joint_vel = np.insert(joint_vel, len(joint_vel), 0)

        joint_command = self.joint_command_init()
        joint_command.velocity = np.ndarray.tolist(joint_vel)
        joint_command.header.stamp = rospy.Time.now()
        self.pub_joint_ctrl_msg(joint_command)
        self._rate.sleep()


    def open_gripper(self):
        self.gripper.open()


    def close_gripper(self):
        self.gripper.close()

    
    def gen_path(current_pose, desired_pose, num_points=100):

        path = np.empty((num_points, 3))
        s = rtb.trapezoidal(0, 1, num_points).s
        for i in range(num_points):
            path[i, :] = (1 - s[i])*current_pose.A[0:3, 3] + \
                s[i]*desired_pose.A[0:3, 3]

        path_to_send = []
        for pose in path:
            p = sm.SE3(pose)
            p.A[:3, :3] = current_pose.A[:3, :3]
            path_to_send.append(p)

        return path_to_send


def main():

    rospy.init_node('mission_replication_node')

    # initialize robot and controller
    robot = Limb()
    controller = robot_control(robot)

    # HOME POSITION
    controller.go_to_joint_angles(WAIT_JS)
    controller.open_gripper()

    # APPROACH PLATE LOCATION
    current_pose = controller.get_ee_pose()
    desired_pose = sm.SE3(0, -0.18, 0.0) @ current_pose
    path_to_send = rtb.ctraj(current_pose, desired_pose, 100)
    controller.follow_cart_path(path_to_send, speed=1)

    # PICK THE PLATE
    controller.close_gripper()
    rospy.sleep(0.5)

    # LIFT THE PALTE
    current_pose = controller.get_ee_pose()
    desired_pose = sm.SE3(0, 0.0, 0.1) @ current_pose
    path_to_send = rtb.ctraj(current_pose, desired_pose, 100)
    controller.follow_cart_path(path_to_send, speed=1)
    rospy.sleep(0.5)

    # MOVE BACKWARD
    current_pose = controller.get_ee_pose()
    desired_pose = sm.SE3(0, 0.2, 0.0) @ current_pose
    path_to_send = robot_control.gen_path(current_pose, desired_pose)
    controller.follow_cart_path(path_to_send, speed=1)

    # # MOVE TO THE RIGHT
    current_pose = controller.get_ee_pose()
    desired_pose = sm.SE3(0.3, 0, 0) @ current_pose
    print(desired_pose)
    path_to_send = robot_control.gen_path(current_pose, desired_pose)
    controller.follow_cart_path(path_to_send, speed=1)

    # MOVE PLATE TO HANGED POSE
    controller.go_to_joint_angles(HANG_TO_BEND)

    # TILE THE PLATE
    current_pose = controller.get_ee_pose()
    desired_pose = current_pose @ sm.SE3.Rz(np.pi/4)
    path_to_send = rtb.ctraj(current_pose, desired_pose, 100)
    controller.follow_cart_path(path_to_send, speed=1)
    rospy.sleep(0.5)

    # UNTILE THE PLATE
    controller.go_to_joint_angles(HANG_TO_BEND)

    # MOVE PLATE TO RETURN POSE
    current_pose = controller.get_ee_pose()
    desired_pose = sm.SE3(0,-0.2, 0) @ current_pose
    path_to_send = rtb.ctraj(current_pose, desired_pose, 100)
    controller.follow_cart_path(path_to_send, speed=1)

    # HANG TO RETURN
    controller.go_to_joint_angles(HANG_TO_RETURN)
    rospy.sleep(0.5)

    controller.go_to_joint_angles(PICK_JS)
    rospy.sleep(0.5)

    # RELEASE THE PLATE
    controller.open_gripper()
    rospy.sleep(0.5)

    # RESET TO WAIT POSE
    controller.go_to_joint_angles(WAIT_JS)



if __name__ == "__main__":
    main()
