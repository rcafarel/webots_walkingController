
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import warnings

def getChain(leg):
    warnings.filterwarnings("ignore", message="Link")
    links = []
    links.append(OriginLink())
    links.append(URDFLink(name="toLegTranslate",  joint_type='fixed',
                          origin_orientation=[leg.robotQ.getRoll(), leg.robotQ.getPitch(), 0.0],
                          origin_translation=[leg.robotPosition.getX() + leg.positionX, leg.robotPosition.getY() + leg.positionY, leg.robotPosition.getZ()]))
    links.append(URDFLink(name="toLegRotate",  joint_type='fixed',
                          origin_translation=[0, 0, 0],
                          origin_orientation=[0, 0, leg.orientationZ.theta]))

    for i in range(leg.numberOfServos):
        links.append(URDFLink(name="translate" + str(i), joint_type='fixed',
                              origin_orientation=[0, 0, 0],
                              origin_translation=[leg.legSegmentArray[i].getX(),
                                                  leg.legSegmentArray[i].getY(),
                                                  leg.legSegmentArray[i].getZ()]))
        links.append(URDFLink(name="rotate" + str(i), joint_type='fixed',
                              origin_translation=[0, 0, 0],
                              origin_orientation=[leg.servoOrientationArray[i].getRoll(),
                                                  leg.servoOrientationArray[i].getPitch(),
                                                  leg.servoOrientationArray[i].getYaw()]))
        links.append(URDFLink(name="servo" + str(i),
                              origin_translation=[0, 0, 0],
                              origin_orientation=[0, 0, 0],
                              rotation=[0, 0, 1]))

    links.append(URDFLink(name="lowerLeg",  joint_type='fixed',
                          origin_orientation=[0, 0, 0],
                          origin_translation=[leg.lowerLegSegment.getX(),
                                              leg.lowerLegSegment.getY(),
                                              leg.lowerLegSegment.getZ()]))

    return Chain(name='defaultLeg', links=links)

def createDHChain_noDH(x, y, theta, dhSegments):
    warnings.filterwarnings("ignore", message="Link")
    links = []
    links.append(OriginLink())
    links.append(URDFLink(name="toLegTranslate",  joint_type='fixed',
                          origin_orientation=[0.0, 0.0, 0.0],
                          origin_translation=[x, y, 0.0]))
    links.append(URDFLink(name="toLegRotate",  joint_type='fixed',
                          origin_translation=[0.0, 0.0, 0.0],
                          origin_orientation=[0.0, 0.0, theta]))
    for i in range(len(dhSegments)):
        dhSegment = dhSegments[i]

        if i < len(dhSegments)-1:
            links.append(URDFLink(name="dhTheta" + str(i),  joint_type='fixed',
                                  origin_orientation=[0.0, 0.0, dhSegment.theta],
                                  origin_translation=[0.0, 0.0, 0.0]))
            links.append(URDFLink(name="dhDAAlpha" + str(i),  joint_type='fixed',
                                  origin_orientation=[dhSegment.alpha, 0.0, 0.0],
                                  origin_translation=[dhSegment.a, 0.0, dhSegment.d]))
            links.append(URDFLink(name="servo" + str(i),
                                  origin_translation=[0.0, 0.0, 0.0],
                                  origin_orientation=[0.0, 0.0, 0.0],
                                  rotation=[0.0, 0.0, 1.0])) # this link is the servo, it is not a fixed link
        else:
            links.append(URDFLink(name="dhAll" + str(i),  joint_type='fixed',
                                  origin_orientation=[0.0, 0.0, dhSegment.theta],
                                  origin_translation=[0.0, 0.0, 0.0]))
            links.append(URDFLink(name="dhAll2" + str(i),  joint_type='fixed',
                                  origin_orientation=[0.0, 0.0, 0.0],
                                  origin_translation=[dhSegment.a, 0.0, dhSegment.d]))

    return Chain(name='dhLeg', links=links)

def getEndEffectorSequence_extraShortSequence(leg, startContactPosition, endContactPosition, endEffectorLiftDeviation):
    contactPositions = 3

    positionArray = []
    contactPositionArray = []
    xDev = (endContactPosition[0] - startContactPosition[0]) / (contactPositions - 1)
    yDev = (endContactPosition[1] - startContactPosition[1]) / (contactPositions - 1)
    zDev = (endContactPosition[2] - startContactPosition[2]) / (contactPositions - 1)
    for i in range(contactPositions):
        nextX = startContactPosition[0] + xDev * i
        nextY = startContactPosition[1] + yDev * i
        nextZ = startContactPosition[2] + zDev * i
        contactPositionArray.append([nextX, nextY, nextZ])
        positionArray.append(contactPositionArray[i])

    nextX = endContactPosition[0] + (startContactPosition[0] - endContactPosition[0]) / 2.0
    nextY = endContactPosition[1] + (startContactPosition[1] - endContactPosition[1]) / 2.0
    nextZ = endContactPosition[2] + endEffectorLiftDeviation
    positionArray.append([nextX, nextY, nextZ])

    # this outputs first and last position as the same
    if not leg.firstTripod: # switched first and second tripod because end effector position was causing the robot to rotate as it was initializing the gait
        positionArray.append(contactPositionArray[0])
        for i in range(len(positionArray)):
            print(positionArray[i])
        return positionArray
    else:
        secondTripodPositionArray = []
        secondTripodPositionArray.append(positionArray[2])
        secondTripodPositionArray.append(positionArray[3])
        secondTripodPositionArray.append(positionArray[0])
        secondTripodPositionArray.append(positionArray[1])
        secondTripodPositionArray.append(positionArray[2])
        for i in range(len(secondTripodPositionArray)):
            print(secondTripodPositionArray[i])
        return secondTripodPositionArray
