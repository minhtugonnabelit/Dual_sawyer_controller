    # # get the plate pose that related to base frame
    # plate_pose = controller.get_ee_pose() @ PICKER_GRIP_POSE.inv()
    # plate = Plate(plate_pose)

    # # interpolate the bending motion
    # step = 50
    # all_seg = []
    # s = rtb.trapezoidal(0, np.deg2rad(12), step).s
    # for i in range(step):

    #     seg_array = []
    #     _pick, _bend = plate.bend(s[i], seg_array)

    #     # somehow this copy is real necessary, otherwise the pose will be updated
    #     pick = copy.deepcopy(_pick)
    #     bend = copy.deepcopy(_bend)

    #     # assign relative orientation of the gripper in plate center frame to the extracted edges pose
    #     pick_ori = pick.A[0:3,0:3] @ PICKER_GRIP_POSE.A[0:3,0:3]
    #     pick.A[0:3,0:3] = pick_ori

    #     # position of the grasping pose is kept
    #     picker_grip_pose = pick

    #     # send motion command
    #     qdot = controller.single_step_control(picker_grip_pose, 0.02)
    #     controller.send_vel_command(qdot)
    #     all_seg.append(seg_array)

    # # Reverse the bending motion
    # step = len(all_seg)
    # for i, seg_array in enumerate(reversed(all_seg)):

        # _pick, _bend = plate.unbend(seg_array)

        # # as noted above
        # pick = copy.deepcopy(_pick)
        # bend = copy.deepcopy(_bend)

        # # assign relative orientation of the gripper in plate center frame to the extracted edges pose
        # pick_ori = pick.A[0:3,0:3] @ PICKER_GRIP_POSE.A[0:3,0:3]
        # pick.A[0:3,0:3] = pick_ori

        # # position of the grasping pose is kept
        # picker_grip_pose = pick

        # # send motion command
        # controller.single_step_control(picker_grip_pose, 0.01)
        # controller.send_vel_command(qdot)
