from kinematics.RobotChain import getEndEffectorSequence_extraShortSequence

class FindWalkingPoses:

    def __init__(self, legs, nCycles, footContactLength):
        self.legs = legs
        self.gaitArray = []
        self.subCommands = []

        self.calculateGait(footContactLength) # generate the actuator sequences for the gait
        self.makeSubCommands(nCycles) # perform n cycles of the actuator sequence

    def calculateGait(self, footContactLength):
        # print("Calculate gait positions")
        positionArray = []

        for legIndex in range(6):
            leg = self.legs[legIndex]

            startContactPosition = [leg.xWalkingCentroid, leg.yWalkingCentroid+footContactLength/2, -90]
            endContactPosition = [leg.xWalkingCentroid, leg.yWalkingCentroid-footContactLength/2, -90]
            endEffectorLiftDeviation = 30

            positionArray.append(getEndEffectorSequence_extraShortSequence(leg, startContactPosition, endContactPosition, endEffectorLiftDeviation))

        sequenceLength = len(positionArray[0])
        for sequenceIndex in range(sequenceLength):
            servoArray = []
            for legIndex in range(6):
                leg = self.legs[legIndex]
                p = positionArray[legIndex][sequenceIndex]
                ik = leg.getIKJointAngles(p[0], p[1], p[2], leg.currentJointAngles)
                servoArray.append(ik[5])
                servoArray.append(ik[8])
                servoArray.append(ik[11])
            self.gaitArray.append(servoArray)
            print(servoArray)

    def makeSubCommands(self, numberOfIterations):
        previousPositions = self.gaitArray[0]
        self.subCommands.append(previousPositions)

        numberOfIntervals = 10

        for n in range(numberOfIterations):
            for commandIndex in range(1, len(self.gaitArray)):
                positionDiffs = []

                for i in range(18):
                    positionDiffs.append((self.gaitArray[commandIndex][i] - previousPositions[i]) / numberOfIntervals)

                self.subCommands.append(previousPositions)

                counter = 0
                while counter <= numberOfIntervals:
                    currentPositions = []
                    for i in range(18):
                        currentPositions.append(previousPositions[i] + positionDiffs[i])
                    counter += 1
                    self.subCommands.append(currentPositions)
                    previousPositions = currentPositions

