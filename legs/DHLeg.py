from kinematics import RobotChain
from legs.DHSegment import DHSegment


class DHLeg:

    def __init__(self, id, x, y, theta, xWalkingCentroid, yWalkingCentroid, firstTripod, dhSegments):
        self.id = id
        self.x = x
        self.y = y
        self.theta = theta
        self.xWalkingCentroid = xWalkingCentroid
        self.yWalkingCentroid = yWalkingCentroid

        self.firstTripod = firstTripod
        self.currentJointAngles = [0]*14

        self.dhSegments = dhSegments
        self.inverseKinematicChain = RobotChain.createDHChain_noDH(self.x, self.y, self.theta, self.dhSegments)

    def addDHSegment(self, theta, d, a, alpha):
        self.dhSegments.append(DHSegment(theta, d, a, alpha))

    def initializeIKChain(self):
        self.inverseKinematicChain = RobotChain.createDHChain_noDH(self.x, self.y, self.theta, self.dhSegments)

    def getIKJointAngles(self, x, y, z, curJointAngles):
        targetFrame = [[1.0, 0.0, 0.0, x],
                       [0.0, 1.0, 0.0, y],
                       [0.0, 0.0, 1.0, z],
                       [0.0, 0.0, 0.0, 1.0]]

        return self.inverseKinematicChain.inverse_kinematics_frame(targetFrame, initial_position=curJointAngles,
                                                                   orientation_mode=None, no_position=False)
