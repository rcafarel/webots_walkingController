"""walkingController controller."""
from controller import Robot
from RobotForWalking import RobotForWalking
from kinematics.FindWalkingPoses import FindWalkingPoses

robot = Robot()
controllerRobot = RobotForWalking() # this is the digital twin used for forward and inverse kinematics

numberOfStrides = 30
footContactLength = 50
walkingPoses = FindWalkingPoses(controllerRobot.getLegs(), numberOfStrides, footContactLength) # calculate servo sequences to perform the walking gait n times
subCommands = walkingPoses.subCommands


# get the time step of the current world.
# timestep is defined as 50 milliseconds, defined on the WorldInfo
timestep = int(robot.getBasicTimeStep())

# define each servo which will be used with the position controller
# both robots have the same servos for uninjured legs
# the last three servos are named differently for the two robots
servos = {}
dhServos = {}
for i in range(18):
    servos[i+1] = robot.getDevice('servo' + str(i+1) + 'Motor')

dhServos[16] = robot.getDevice('dh_servo1')
dhServos[17] = robot.getDevice('dh_servo2')
dhServos[18] = robot.getDevice('dh_servo3')


commandCounter = 0

# Main loop:
# - perform simulation steps until Webots is stopping the controller
# - or we reach the end of our subcommands
while robot.step(timestep) != -1 and commandCounter < len(subCommands):

    subCommand = subCommands[commandCounter]

    for i in range(18):
        position = subCommand[i]
        if type(position) == 'float': # if the command comes in as a number
            if servos[i+1] is not None:
                servos[i+1].setPosition(position)
            if i >= 15:
                if dhServos[i+1] is not None:
                    dhServos[i+1].setPosition(position)
        else: # otherwise it will come in as a numpy float which needs to be converted
            if servos[i+1] is not None:
                servos[i+1].setPosition(float(position))
            if i >= 15:
                if dhServos[i+1] is not None: # this is for the robot with replaced leg to represent the dh parameters
                    dhServos[i+1].setPosition(float(position))


    commandCounter += 1


