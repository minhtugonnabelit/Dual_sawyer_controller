
    # def home_robot(self):
    #     if self._reset:
    #         self._limb.move_to_neutral()
    #         rospy.loginfo('Robot homed!')
    #         self._reset = not self._reset

    # def joint_angles_control_init(self):

    #     try:
    #         self.traj = MotionTrajectory(limb = self._limb)

    #         wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=0.3,
    #                                         max_joint_accel=0.05)
    #         self.waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = self._limb)

    #         self.joint_angles = self._limb.joint_ordered_angles()

    #         self.waypoint.set_joint_angles(joint_angles = self.joint_angles)
    #         self.traj.append_waypoint(self.waypoint.to_msg())

    #     except rospy.ROSInterruptException:
    #         rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.') 

    # def go_to_joint_angles(self):
    #     flag = True
    #     while not rospy.is_shutdown():

    #         self._pub_rate.publish(5)
    #         if self._ext.get_js_flag():
    #         # if flag:

    #             js = self._ext.get_js()
    #             desired_js = deque(js.position)
    #             desired_js.pop()
    #             desired_js.popleft()
    #             print(self.joint_angles)
    #             print(js)

    #             if len(desired_js) != len(self.joint_angles):
    #                 rospy.logerr('The number of joint_angles must be %d', len(self.joint_angles))
    #                 return None

    #             self.waypoint.set_joint_angles(joint_angles = desired_js)
    #             self.traj.append_waypoint(self.waypoint.to_msg())

    #             result = self.traj.send_trajectory()
    #             if result is None:
    #                 rospy.logerr('Trajectory FAILED to send')
    #                 return

    #             if result.result:
    #                 rospy.loginfo('Motion controller successfully finished the trajectory!')
    #             else:
    #                 rospy.logerr('Motion controller failed to complete the trajectory with error %s',
    #                         result.errorId)
                    
    #             self.traj.clear_waypoints()
    #             flag = False

                
    #         else:
    #             rospy.loginfo("waiting for signal to execute control")
            
    #         self._rate.sleep()

    # def cartersian_pose_init(self):
    #     try:

    #         # Init traj options to CARTESIAN
    #         self.traj_options = TrajectoryOptions()
    #         self.traj_options.interpolation_type = TrajectoryOptions.CARTESIAN

    #         # Init traj with traj options and limb
    #         self.traj = MotionTrajectory(trajectory_options = self.traj_options, limb = self._limb)

    #         # Waypoint options initialize
    #         wpt_opts = MotionWaypointOptions(max_linear_speed = 0.6,
    #                                         max_linear_accel = 0.6,
    #                                         max_rotational_speed = 1.57,
    #                                         max_rotational_accel = 1.57,
    #                                         max_joint_speed_ratio = 1.0)
            
    #         # Waypoint initialize
    #         self.waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = self._limb)

    #     except rospy.ROSInterruptException:
    #         rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.') 

    # def set_waypoint(self, pose):
    #     """
    #     Set pose from sm.SE3 to posestamped type"""
    #     endpoint_state = self._limb.tip_state(self._tip_name)
    #     if endpoint_state is None:
    #         rospy.logerr('Endpoint state not found with tip name %s', self._tip_name)
    #         return None
    #     waypoint = endpoint_state.pose

    #     # directly set the pose translation part
    #     # waypoint.position.x = pose.A[0,3]
    #     # waypoint.position.y = pose.A[1,3]
    #     # waypoint.position.z = pose.A[2,3]

    #     waypoint.position.x = pose[0]
    #     waypoint.position.y = pose[1]
    #     waypoint.position.z = pose[2]

    #     # convert the rotation matrix to quaternion
    #     # pose_ori_quat = smb.r2q(pose.A[:3,:3], order='xyzs').T

    #     # assign the quaternion to the pose
    #     waypoint.orientation.x = pose[3]
    #     waypoint.orientation.y = pose[4]
    #     waypoint.orientation.z = pose[5]
    #     waypoint.orientation.w = pose[6]

    #     poseStamped = PoseStamped()
    #     poseStamped.pose = waypoint

    #     joint_angles = self._limb.joint_ordered_angles()
    #     self.waypoint.set_cartesian_pose(poseStamped, self._tip_name, joint_angles)

    #     return self.waypoint
    
    # def gen_traj_from_c_waypoint(self, path):

    #     for pose in path:
    #         self.set_waypoint(pose)
    #         self.traj.append_waypoint(self.waypoint.to_msg())

    # def follow_Cartesian_path(self, path)-> bool:

    #     self.Done = False
    #     self.gen_traj_from_c_waypoint(path)

    #     result = self.traj.send_trajectory()
    #     if result is None:
    #         rospy.logerr('Trajectory FAILED to send')
    #         return

    #     if result.result:
    #         rospy.loginfo('Motion controller successfully finished the trajectory!')
    #     else:
    #         rospy.logerr('Motion controller failed to complete the trajectory with error %s',
    #                     result.errorId)
        
    #     self.Done = result.result        
    #     self.traj.clear_waypoints()

    #     return self.Done
    
    # def go_to_cart_pose(self, pose):
        
    #     self.set_waypoint(pose)
    #     self.traj.append_waypoint(self.waypoint.to_msg())
    #     result = self.traj.send_trajectory()
    #     if result is None:
    #         rospy.logerr('Trajectory FAILED to send')
    #         return

    #     if result.result:
    #         rospy.loginfo('Motion controller successfully finished the trajectory!')
    #     else:
    #         rospy.logerr('Motion controller failed to complete the trajectory with error %s',
    #                     result.errorId)
        
    #     self.Done = result.result        
    #     self.traj.clear_waypoints()

    #     return self.Done

