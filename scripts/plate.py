from spatialmath import SE3
import spatialgeometry as geometry
import os
import copy
from math import pi

class Plate():
    """
    Class for simulating the printer's plate
    The frame of each segment is at its center
    
    The Plate should be moved in flat state
    After moved to dropping location, perform tilting also with move_flat_plate 
    Then conduct bending, unbend and tilt back 

    Check out plat_test.py
    """

    FULL_DIST = 0.22
    SEGMENT_NUM = 11 # odd num

    SEGMENT_LENGTH = FULL_DIST / SEGMENT_NUM
    MID_TO_EDGE = SEGMENT_LENGTH * int((SEGMENT_NUM-1)/2)

    _script_directory = os.path.dirname(os.path.abspath(__file__))

    
    def __init__(self, pose : SE3):
        """
        _pose: the middle segment's pose represents the whole plate's pose
        """
        self._pose = pose
        file_name = os.path.join(os.path.abspath(os.path.dirname(__file__)),'mesh','plate.dae')
        
        self._segments = [geometry.Mesh(file_name, color = (1,95/255,31/255,1)) for i in range(Plate.SEGMENT_NUM)]
        
        self.move_flat_plate(pose)


    def get_pose(self):
        return self._pose

    def move_flat_plate(self, pose : SE3, part_relate = True):
        """
        Update new position, update as internal state  
        pose : middle segment
        first_seg_pose : position to grip
        """
        self._pose = pose
        
        first_seg_pose = pose * SE3.Trans(-Plate.MID_TO_EDGE,0,0) 
        seg_pose = first_seg_pose
        for seg in self._segments:
            seg.T = seg_pose
            seg_pose *= SE3.Trans(Plate.SEGMENT_LENGTH,0,0)

        return first_seg_pose


    def bend(self, increment, seg_array):
        """
        Bend the plate
        Return the pose of 2 outer segments
        Shoule be further converted to gripper poses 
        """

        # The anchor point stays staionary during bending
        middle_segment_number = int((Plate.SEGMENT_NUM-1)/2) #5

        # Pose of 2 gripper
        picker_pose = None
        bender_pose = None

        # Bending parameters        
        # Increment is the increment of seg_pose after each step
        # INCREMENT = pi/300 * i
        
        seg_pose = self._pose
        for seg in self._segments[:middle_segment_number]:
            seg_pose = seg_pose * SE3.Trans(Plate.SEGMENT_LENGTH/2,0,0) * SE3.Ry(increment) * SE3.Trans(Plate.SEGMENT_LENGTH/2,0,0)
            seg.T = seg_pose     
            seg_array.append(seg_pose)
        picker_pose = seg_pose

        # Middle segment
        seg_array.append(self._pose)

        seg_pose = self._pose
        for seg in self._segments[(middle_segment_number+1):]:
            seg_pose = seg_pose * SE3.Trans(-Plate.SEGMENT_LENGTH/2,0,0) * SE3.Ry(-increment) * SE3.Trans(-Plate.SEGMENT_LENGTH/2,0,0)
            seg.T = seg_pose   
            seg_array.append(seg_pose)      
        bender_pose = seg_pose

        return [picker_pose, bender_pose]


    def unbend(self, seg_array):

        middle_segment_number = int((Plate.SEGMENT_NUM-1)/2) #5
        for index_seg, seg in enumerate(self._segments):
            seg.T = seg_array[index_seg]

        
        #   Get the poses of the two outer segments
        picker_pose = seg_array[middle_segment_number-1]
        bender_pose = seg_array[-1]

        return [picker_pose, bender_pose]

        


if __name__ == "__main__":
    pass

    