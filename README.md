# frc-2023
Code for the 2023 FRC Season\
[Mission Statement](https://docs.google.com/document/d/1II30-4GxlCg0nSo4__XucbIbuxTqHBn9H1TNUuyu37s/edit?usp=sharing)\
[Protocol](https://docs.google.com/document/d/1I5Eu0LzLJGGpliEfknBvpgCctkRO0zkXJXP7kYwBYbg/edit?usp=sharing)\
[Request to Collaborate](https://docs.google.com/forms/d/e/1FAIpQLSfmF3fr5YMJInckkCB_NlD0ZAlc73mYTkqMPMI7dA8ExYZ00g/viewform?usp=sf_link)

## How to Contribute:
1. Fill out the collaboration form to get added to the repository.
2. If the subsystem you want to work on doesn't have a branch yet, create a new branch for it.
3. Once your code is written, create a pull request so it can be reviewed.

## Subsystems (hardware)
- Drive Base: 4-falcon tank drive. Square chassis. custom 2-speed ball gearboxes (with ball shifters), high speed 10.66:1, low speed 17.88
- Turret: 1 falcon through a 100:1 versaplanetary and a 5:1 driving gear (total 500:1)
- Pivot: 2 falcons through a (tbd) versaplanetary with a (tbd) chain gear reduction
- Arm: 3-stage igus slide elevator driven by a single falcon through a (tbd) versaplanetary. Belt reduces linear motion to (tbd) m/rotation
- Wrist: Keeps the end effector parralell to the floor, and flips the end effector to keep cones upright. Driven by a single falcon on a versaplanetary, and a 180 degree rotary pneumatic actuator
- End Effector: horizontal end effector with 4 driven compliant wheels. One neo 550 with a (tbd) belt reduction on each side. Has a beam brake sensor and a color sensor to keep track of gamepieces. Can open for cubes and close to grab cones with a pneumatic cylinder.
- Dual limelights: One on the drivetrain, primarily for apriltags. The other on the end effector to assist in placing cones, using the retroreflective tape.

## Technical Features
- Drive base
  - Path following using a state space controller for velocity control and a linear time-varying unicycle controller to follow trajectories.
  - Shifting coasts motors breifly to minimize turning because of one gearbox engaging before the other.
  - Multiple drive modes
  - Odometry fused with apriltag data from limelight, with outlier rejection to ensure robust data.
- Supersystem (turret, pivot, arm, wrist)
  - Kinematics to move the end effector to any 3d point
  - State-space control of all actuators, using custom trapezoidal motion profiles that are robust enough to be correct even when re-generated every loop, allowing the supersystem setpoint to be updated in real time.
  - Can be used with odometry to put the end effector at a field-relative 3d coordinate.
- End Effector
  - Uses rev color sensor v3 to determine whether it is holding a cube or a cone.
  - Uses beam brake seonsor to autmatically grab cones as soon as they are in the intake

## Interface (plan... in progress)
Driver:
- Flight stick controller, curvature based drive or arcade drive (depending on driver preference)
- Thumb button actuates gearboxes
Operator:
- XBox-style controller
- Buttons for placing low, mid, or high. Selects nearest location in the given level to place at. Automatially chooses cone/cube locations depending on color sensor.
- Button to intake a cube, button to intake a cone. Defaults to aiming for substation.
  - use POV hat to specify a direction to switch to ground pickup
