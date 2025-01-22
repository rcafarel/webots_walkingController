This program is defined to be used as the robot controller in Webots. The robot it controls is a hexapod with 18 degrees
of freedom (3 for each leg). The configuration of each leg is described in the classes:
RobotForWalking.py
Leg.py
DHLeg.py

Leg.py contains the default configurations for each leg model using full 3D translations and quaternion rotations.
DHLeg.py is the DH representation of a leg. The DH parameters for an injured leg (currently only the front right leg)
are stored (and can be modified) within the PriorDHLegModels.py class.

After modifying the leg parameters the Webots world can be reloaded and the simulation can be run.