

import rospy
from std_msgs.msg import (Bool, UInt16, String)
import intera_interface

from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)

from intera_motion_interface import MotionControllerActionClient

import spatialmath.base as smb
import spatialmath as sm
import roboticstoolbox as rtb

# MOTION CONTROLLER METHOD CLASS


class Robot_control():

    def __init__(self, limb):

        # rospy.loginfo('Robot homing!')
        self._limb = limb
        # self._limb.move_to_neutral()
        # rospy.loginfo('Robot homed!')

        self._tip_name = 'right_hand'
        self._rate = rospy.Rate(5)
        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)

    def reset_state(self):
        self._reset = True

    def home_robot(self):
        # if self._reset:
        # rospy.loginfo('Robot homing!')
        rospy.loginfo('Robot homing!')
        self._limb.move_to_neutral()
        rospy.loginfo('Robot homed!')
        # self._reset = not self._reset

    def joint_angles_control_init(self):

        try:
            self.traj = MotionTrajectory(limb=self._limb)

            wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=0.3,
                                            max_joint_accel=0.05)
            self.waypoint = MotionWaypoint(
                options=wpt_opts.to_msg(), limb=self._limb)

            self.joint_angles = self._limb.joint_ordered_angles()

            self.waypoint.set_joint_angles(joint_angles=self.joint_angles)
            self.traj.append_waypoint(self.waypoint.to_msg())

        except rospy.ROSInterruptException:
            rospy.logerr(
                'Keyboard interrupt detected from the user. Exiting before trajectory completion.')

    def go_to_joint_angles(self, desired_q):

        if len(desired_q) != len(self.joint_angles):
            rospy.logerr('The number of joint_angles must be %d',
                         len(self.joint_angles))
            return None

        self.waypoint.set_joint_angles(joint_angles=desired_q)
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


sawyer = rtb.models.DH.Sawyer()
# print(sawyerq_q.qz)


rospy.init_node('go_to_joint_angle')

robot = intera_interface.Limb()

motion = Robot_control(robot)
motion.home_robot()
motion.joint_angles_control_init()

desired_q = [-1.5133369140625, 
            -0.5920302734375, 
            -0.216025390625,
            1.26565234375, 
            0.152361328125, 
            0.9298896484375, 
            3.20494921875]

sawyer.q = desired_q
motion.go_to_joint_angles(desired_q)


block = sawyer.fkine(sawyer.q)
# print(block)

lifting = sm.SE3(0,0,0.3).A @ block.A
lift_q = sawyer.ikine_LM(lifting, q0=desired_q, joint_limits=True, mask=(0,0,0,1,1,1)).q

motion.go_to_joint_angles(lift_q)
