from legs.Leg import Leg
import math
from legs import PriorDHLegModels


class RobotForWalking:

    def __init__(self):
        print("Initialize left legs")
        self.backLeftLeg = Leg("backLeftLeg", -39.5, -100.0, 3.0 * math.pi / 4.0, [1, 2, 3], True, True, -200.0, -180.0)
        self.middleLeftLeg = Leg("middleLeftLeg", -64.5, 0.0, math.pi / 2.0, [4, 5, 6], False, True, -250.0, 0)
        self.frontLeftLeg = Leg("frontLeftLeg", -39.5, 100.0, math.pi / 4.0, [7, 8, 9], True, True, -200.0, 180.0)

        print("Initialize right legs")
        self.backRightLeg = Leg("backRightLeg", 39.5, -100.0, -3.0 * math.pi / 4.0, [10, 11, 12], False, False, 200.0, -180.0)
        self.middleRightLeg = Leg("middleRightLeg", 64.5, 0.0, -1.0 * math.pi / 2.0, [13, 14, 15], True, False, 250.0, 0)
        # this is the original uninjured leg, the DH leg takes its place for walking with the new proprioceptive model
        # self.frontRightLeg = Leg("frontRightLeg", 39.5, 100.0, -1.0 * math.pi / 4.0, [16, 17, 18], False, False, 200.0, 180.0)
        self.frontRightLeg = PriorDHLegModels.frontRightLeg

    def getLegs(self):
        return [self.backLeftLeg, self.middleLeftLeg, self.frontLeftLeg, self.backRightLeg, self.middleRightLeg, self.frontRightLeg]

    def getFirstTripodLegs(self):
        return [self.backLeftLeg, self.frontLeftLeg, self.middleRightLeg]

    def getSecondTripodLegs(self):
        return [self.middleLeftLeg, self.backRightLeg, self.frontRightLeg]