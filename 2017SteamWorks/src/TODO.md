# TODO

### Erryday
Tune autos
	* Near Hopper Inside
	* Near Hopper Outside
	* Near Hopper Receive

### With the Bob
Implement Swerve current detection to know when we've run into something
Lock shooter speed (well) before the shooter begins spinning
Troubleshoot the vision oscillating to find target, even though the pre-defined angles just jump to
Look into PID (incl. I) for turret: goal is +-1 degree

### Without the Bob
//Reboot `com.team254.cheezdroid` through ADB
Develop more complex Arduino LED controls
Develop LightController class
Add serial communications for lights
//Add co-driver controls to bump up/down shooter RPM manually
Fix math for vision abscissa

## Doing
Change logic on when to apply power to Swerve drive modules based on all their angles, rather than each individual module's angle
Fix vision position math
Tune Near Hopper Inside auto
Troubleshoot the vision oscillating to find target, even though the pre-defined angles just jump to