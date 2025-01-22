import numpy

from kinematics import RobotChain
from matrix.Quaternion import Quaternion
from matrix.RotationZ import RotationZ
from matrix.Translation import Translation


class Leg:

    # assume servoIds will contain at least one id, test case has 3
    def __init__(self, id, x, y, orientationZ, servoIds, firstTripod, onLeft, xWalkingCentroid, yWalkingCentroid):
        self.id = id
        self.robotPosition = Translation([0.0, 0.0, 0.0])
        self.robotQ = Quaternion([1.0, 0.0, 0.0, 0.0])

        self.positionX = x
        self.positionY = y
        self.orientationZ = RotationZ(orientationZ)

        self.firstTripod = firstTripod
        self.onLeft = onLeft
        self.xWalkingCentroid = xWalkingCentroid
        self.yWalkingCentroid = yWalkingCentroid

        self.legReferenceFrameMatrix = None
        self.legReferenceFramePosition = None

        self.numberOfServos = len(servoIds)

        self.legSegmentArray = numpy.empty(self.numberOfServos, dtype=Translation)
        self.servoOrientationArray = numpy.empty(self.numberOfServos, dtype=Quaternion)

        self.servoReferenceFrameArray = []
        self.servoEndReferenceFrameArray = []
        self.servoPositionArray = []

        for i in range(self.numberOfServos):
            self.legSegmentArray[i] = Translation([0.0, 0.0, 0.0])  # mm
            self.servoOrientationArray[i] = Quaternion([0.0, 0.0, 0.0, 0.0])  # quaternions
            self.servoReferenceFrameArray.append(None)
            self.servoEndReferenceFrameArray.append(None)
            self.servoPositionArray.append(None)

        self.lowerLegSegment = Translation([0.0, 0.0, 0.0])  # mm

        self.footMatrix = None
        self.footPosition = None

        self.currentJointAngles = [0]*(4 + self.numberOfServos*3)
        self.inverseKinematicChain = None
        self.setDefaultParameters()

    def setDefaultParameters(self):
        self.legSegmentArray[0] = Translation([0.0, 27.0, 0.0])
        self.legSegmentArray[1] = Translation([0.0, 44.0, 0.0])
        self.legSegmentArray[2] = Translation([0.0, 75.0, 0.0])

        # this has a bend, so we probably want to update, 136 was along a hypotenuse in the xy-plane
        # self.lowerLegSegment = Translation([0.0, 136.0, 0.0])
        if self.onLeft:
            self.lowerLegSegment = Translation([101.0, 91, 0.0])
        else:
            self.lowerLegSegment = Translation([-101.0, 91, 0.0])

        self.servoOrientationArray[0] = Quaternion([1.0, 0.0, 0.0, 0.0])  # no rotation from body
        # y-axis -pi / 2 from hipServo, positive for right side
        direction = 1.0
        if self.onLeft:
            direction = -1.0
        self.servoOrientationArray[1] = Quaternion([0.7071, 0.0, direction * 0.7071, 0.0])
        self.servoOrientationArray[2] = Quaternion([0.0, 0.0, 1.0, 0.0])  # y-axis pi from kneeServo

        self.inverseKinematicChain = RobotChain.getChain(self)

    def getIKJointAngles(self, x, y, z, initialPosition):
        targetFrame = [[1.0, 0.0, 0.0, x],
                       [0.0, 1.0, 0.0, y],
                       [0.0, 0.0, 1.0, z],
                       [0.0, 0.0, 0.0, 1.0]]

        return self.inverseKinematicChain.inverse_kinematics_frame(targetFrame, initial_position=initialPosition,
                                                                   orientation_mode=None, no_position=False)


