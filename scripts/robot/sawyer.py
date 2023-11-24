import numpy as np
import roboticstoolbox as rtb

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
    
if __name__ == "__main__":

    robot = Sawyer()
    fig = robot.plot(robot.q, block=False)
    fig.hold()
