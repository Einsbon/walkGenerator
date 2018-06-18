import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import math


class WlakGenerator():
    def __init__(self):
        #                        0   1   2   3   4   5   6   7   8   9  10  11
        self._motorDirection = [+1, +1, +1, -1, +1, +1, +1, -1, -1, +1, -1, -1]
        self._motorDirectionRight = [+1, +1, +1, -1, +1, +1]
        self._motorDirectionLeft = [+1, -1, -1, +1, -1, -1]
        self._walkPoint0 = 0
        self._walkPoint1 = 0
        self._walkPoint2 = 0
        self._walkPoint3 = 0
        #self._walkPoint1f = 0
        #self._walkPoint2f = 0
        self._walkPointStartRight = 0  # 오른쪽 발을 먼저 내미는 것을 기준으로 함. 그때의 오른쪽 발의 지점.
        self._walkPointStartLeft = 0  # 오른쪽 발을 먼저 내미는 것을 기준으로 함. 그때의 왼쪽 발의 지점.
        self._walkPointEndRight = 0  # 오른쪽 발을 디디면서 끝나는 것을 기준으로 함. 그때의 오른쪽 발의 지점.
        self._walkPointEndLeft = 0  # 오른쪽 발을 디디면서 끝나는 것을 기준으로 함. 그때의 왼쪽 발의 지점.

        self._walkPoint0Inverse = 0
        self._walkPoint1Inverse = 0
        self._walkPoint2Inverse = 0
        self._walkPoint3Inverse = 0
        #self._walkPoint1fInverse = 0
        #self._walkPoint2fInverse = 0
        self._walkPointStartInverse = 0
        self._walkPointEndInverse = 0

        self._walkPointStartRightInverse = 0  # 왼쪽으로 sway 했다가 오른발을 먼저 내밈.
        self._walkPointStartLeftInverse = 0  # 오른쪽으로 sway 했다가 왼발을 먼저 내밈.
        self._walkPointEndRightInverse = 0  # 오른발을 디디면서 끝남.
        self._walkPointEndLeftInverse = 0  # 왼발을 디디면서 끝남.

        self._walkPoint0AnkleX = 0
        self._walkPoint1AnkleX = 0
        self._walkPoint2AnkleX = 0
        self._walkPoint3AnkleX = 0

        # 길이 단위: mm
        self._pelvic_interval = 70.5
        self._legUp_length = 110
        self._legDown_length = 110
        self._footJoint_to_bottom = 45

        self._bodyMovePoint = 0
        self._legMovePoint = 0
        self._h = 0
        self._l = 0
        self._sit = 0
        self._swayBody = 0
        self._swayFoot = 0
        self._swayShift = 0
        self._weightStart = 0
        self._weightEnd = 0
        self._swayPlus = 0
        self._walkTime = 0
        self._bodyPositionXPlus = 0
        self._damping = 0

    def setRobotParameter(self, pelvic_interval, leg_up_length, leg_down_length, foot_to_grount, foot_to_heel, foot_to_toe):
        pass

    def setWalkParameter(self, bodyMovePoint, legMovePoint, h, l, sit, swayBody, swayFoot, bodyPositionXPlus, swayShift, weightStart, weightEnd, swayPlus, walkTime, damping, incline):
        self._bodyMovePoint = bodyMovePoint
        self._legMovePoint = legMovePoint
        self._h = h
        self._l = l
        self._sit = sit
        self._swayBody = swayBody
        self._swayFoot = swayFoot
        self._swayShift = swayShift
        self._weightStart = weightStart
        self._weightEnd = weightEnd
        self._swayPlus = swayPlus
        self._walkTime = walkTime
        self._bodyPositionXPlus = bodyPositionXPlus  # +면 몸을 앞으로, -면 몸을 뒤로 하고 걸음.
        self._damping = damping
        self._incline = incline

    def generate(self):
        walkPoint = self._bodyMovePoint*2+self._legMovePoint*2
        walkPointXYZ = np.zeros((4, 3, walkPoint))

        periodStart0 = 0
        periodStart1 = self._bodyMovePoint
        periodStart2 = self._bodyMovePoint+self._legMovePoint
        periodStart3 = self._bodyMovePoint+self._bodyMovePoint+self._legMovePoint

        walkPoint0 = np.zeros((3, self._bodyMovePoint))
        walkPoint1 = np.zeros((3, self._legMovePoint))
        walkPoint2 = np.zeros((3, self._bodyMovePoint))
        walkPoint3 = np.zeros((3, self._legMovePoint))

        walkPointStartRight = np.zeros((3, self._bodyMovePoint+self._legMovePoint))
        walkPointStartLeft = np.zeros((3, self._bodyMovePoint+self._legMovePoint))
        walkPointEndRight = np.zeros((3, self._bodyMovePoint+self._legMovePoint))
        walkPointEIndRight = np.zeros((3, self._bodyMovePoint+self._legMovePoint))

        for i in range(self._bodyMovePoint):
            t = i/(walkPoint-self._legMovePoint)
            walkPoint0[0][i] = -self._l*(t-0.5)
            walkPoint0[2][i] = self._sit
            walkPoint0[1][i] = self._swayBody*math.sin(2 * math.pi*((i-self._swayShift)/walkPoint))

        for i in range(self._legMovePoint):
            t = (i + self._bodyMovePoint)/(walkPoint-self._legMovePoint)
            walkPoint1[0][i] = -self._l*(t-0.5)
            walkPoint1[2][i] = self._sit
            walkPoint1[1][i] = self._swayBody * \
                math.sin(2 * math.pi*((i + self._bodyMovePoint-self._swayShift)/walkPoint))

        for i in range(self._bodyMovePoint):
            t = (i + self._bodyMovePoint+self._legMovePoint)/(walkPoint-self._legMovePoint)
            walkPoint2[0][i] = -self._l*(t-0.5)
            walkPoint2[2][i] = self._sit
            walkPoint2[1][i] = self._swayBody * \
                math.sin(2 * math.pi*((i + self._bodyMovePoint+self._legMovePoint-self._swayShift)/walkPoint))

        for i in range(self._legMovePoint):
            t = i / self._legMovePoint
            sin_tpi = math.sin(t * math.pi)

            walkPoint3[0][i] = (2 * t - 1 + (1-t) * self._weightStart * -sin_tpi +
                                t * self._weightEnd * sin_tpi) * self._l / 2
            walkPoint3[2][i] = math.sin(t * math.pi) * self._h + self._sit
            walkPoint3[1][i] = math.sin(t * math.pi) * self._swayFoot + self._swayBody * \
                math.sin(2 * math.pi*((i+walkPoint-self._legMovePoint-self._swayShift)/walkPoint))

        if self._incline != 0:
            walkPoint0[2] = walkPoint0[2] + walkPoint0[0]*self._incline
            walkPoint1[2] = walkPoint1[2] + walkPoint1[0]*self._incline
            walkPoint2[2] = walkPoint2[2] + walkPoint2[0]*self._incline
            walkPoint3[2] = walkPoint3[2] + walkPoint3[0]*self._incline

        if self._bodyPositionXPlus != 0:
            walkPoint0[0] = walkPoint0[0] - self._bodyPositionXPlus
            walkPoint1[0] = walkPoint1[0] - self._bodyPositionXPlus
            walkPoint2[0] = walkPoint2[0] - self._bodyPositionXPlus
            walkPoint3[0] = walkPoint3[0] - self._bodyPositionXPlus

        if self._damping != 0:
            dampHeight = (walkPoint3[2][-1]-walkPoint0[2][0])/2
            walkPoint0[2][0] = walkPoint0[2][0]+dampHeight*self._damping
            walkPoint2[2][0] = walkPoint2[2][0]-dampHeight*self._damping

        self._walkPoint0 = walkPoint0
        self._walkPoint1 = walkPoint1
        self._walkPoint2 = walkPoint2
        self._walkPoint3 = walkPoint3

        self._walkPointStartRight = walkPointStartRight
        self._walkPointStartLeft = walkPointStartLeft

    def inverseKinematics(self, point, isRightLeg):
        inverseAngle = np.zeros((point[0].size, 6))
        for i in range(point[0].size):
            x = point[0][i]
            y = point[1][i]
            z = point[2][i]

            l3 = self._legUp_length
            l4 = self._legDown_length

            fx = x
            fy = y
            fz = self._legUp_length + self._legDown_length - z

            a = math.sqrt(fx*fx + fy * fy + fz * fz)

            d1 = math.asin(fx/a)
            d2 = math.acos((l3*l3+a*a-l4*l4)/(2*l3*a))
            d3 = math.acos(fz/a)
            d4 = math.acos((l4*l4+a*a-l3*l3)/(2*l4*a))
            d5 = math.pi-d2-d4

            t1 = (math.atan2(fy, fz))
            t2 = d1+d2
            t3 = math.pi-d5
            t4 = -t2+t3
            t5 = -t1

            if isRightLeg:
                inverseAngle[i] = np.array([0, t1, t2, t3, t4, t5]) * self._motorDirectionRight
            else:
                inverseAngle[i] = np.array([0, t1, t2, t3, t4, t5]) * self._motorDirectionLeft

        return inverseAngle

    def showGaitPoint2D(self):
        plt.scatter(self._walkPoint0[0], self._walkPoint0[2])
        plt.scatter(self._walkPoint1[0], self._walkPoint1[2])
        plt.scatter(self._walkPoint2[0], self._walkPoint2[2])
        plt.scatter(self._walkPoint3[0], self._walkPoint3[2])
        plt.show()

    def showGaitPoint3D(self):
        fig = plt.figure(1)
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(self._walkPoint0[0], self._walkPoint0[1], self._walkPoint0[2], 'o')  # , 'bo')
        ax.plot(self._walkPoint1[0], self._walkPoint1[1], self._walkPoint1[2], 'o')  # , 'co')
        ax.plot(self._walkPoint2[0], self._walkPoint2[1], self._walkPoint2[2], 'o')  # , 'go')
        ax.plot(self._walkPoint3[0], self._walkPoint3[1], self._walkPoint3[2], 'o')  # , 'yo')

        ax.plot(self._walkPointStartRight[0], self._walkPointStartRight[1], self._walkPointStartRight[2], 'o')  # , 'yo')
        ax.plot(self._walkPointStartLeft[0], self._walkPointStartLeft[1], self._walkPointStartLeft[2], 'o')  # , 'yo')
        plt.show()

    def inverseKinematicsAll(self):
        walkPoint0Inverse_right = self.inverseKinematics(self._walkPoint0, True)
        walkPoint1Inverse_right = self.inverseKinematics(self._walkPoint1, True)
        walkPoint2Inverse_right = self.inverseKinematics(self._walkPoint2, True)
        walkPoint3Inverse_right = self.inverseKinematics(self._walkPoint3, True)

        walkPoint0Inverse_left = self.inverseKinematics(self._walkPoint0, False)
        walkPoint1Inverse_left = self.inverseKinematics(self._walkPoint1, False)
        walkPoint2Inverse_left = self.inverseKinematics(self._walkPoint2, False)
        walkPoint3Inverse_left = self.inverseKinematics(self._walkPoint3, False)

        self._walkPoint0Inverse = np.column_stack([walkPoint0Inverse_right, walkPoint2Inverse_left])
        self._walkPoint1Inverse = np.column_stack([walkPoint1Inverse_right, walkPoint3Inverse_left])
        self._walkPoint2Inverse = np.column_stack([walkPoint2Inverse_right, walkPoint0Inverse_left])
        self._walkPoint3Inverse = np.column_stack([walkPoint3Inverse_right, walkPoint1Inverse_left])


def main():
    walk = WlakGenerator()
    walk.setWalkParameter(bodyMovePoint=12, legMovePoint=12, h=50, l=130, sit=30, swayBody=55, swayFoot=0,
                          bodyPositionXPlus=0, swayShift=0, weightStart=0.4, weightEnd=0.7, swayPlus=0, walkTime=0.06, damping=0.0, incline=0.0)
    walk.generate()
    walk.showGaitPoint2D()
    walk.showGaitPoint3D()


if __name__ == "__main__":
    main()
