# TODO


### Erryday
Tune autos

1. Near Hopper Inside
2. Near Hopper Outside
3. Near Hopper Receive


### With the Bob
* Implement Swerve current detection to know when we've run into something
* Lock shooter speed (well) before the shooter begins spinning
* Troubleshoot the vision oscillating to find target, even though the pre-defined angles just jump to
* Look into PID (incl. I) for turret: goal is +-1 degree
    * Research "i-zone" calculation for the Talon


### Without the Bob
* <del>Reboot `com.team254.cheezdroid` through ADB
* Develop more complex Arduino LED controls
* Develop LightController class
* Add serial communications for lights
* <del>Add co-driver controls to bump up/down shooter RPM manually
* Fix math for vision abscissa
* Check that auto selection switch has all the appropriate `break` statements, no fallthrough
* Move the alliance selection switch out of the auto-calling switch, assign its output to a variable, and call the autos with that variable
* Fix indentation, especially in the auto selection switch
* Swerve.java, near line 300: Shouldn't the math follow this form?

        `dx = distance * Math.cos(Math.toRadians(90-(intakeAngle + wheelAngle)))`


## Doing
* <del>Change logic on when to apply power to Swerve drive modules based on all their angles, rather than each individual module's angle
* Fix vision position math
* Tune Near Hopper Inside auto
* Troubleshoot the vision oscillating to find target, even though the pre-defined angles just jump to
