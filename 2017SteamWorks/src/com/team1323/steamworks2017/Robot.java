
package com.team1323.steamworks2017;

import ControlSystem.FSM;
import ControlSystem.RoboSystem;
import IO.TeleController;
import SubSystems.DistanceController;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends SampleRobot {
	private RoboSystem robot;
	private TeleController controllers;
	private FSM fsm;
	final String defaultAuto = "off";
	final String one_gear = "one_gear";
	final String two_gear = "two_gear";
	final String near_hopper = "near_hopper";
	SendableChooser autoSelect;
	private DistanceController dist;
	public static enum AUTO {
		OFF,
		ONE_GEAR,
		TWO_GEAR,
		NEAR_HOPPER
	}

	public Robot() {
		autoSelect = new SendableChooser();
		autoSelect.addDefault("Off",defaultAuto);
		autoSelect.addObject("One_Gear",one_gear);
		autoSelect.addObject("Two_Gear",two_gear);
		autoSelect.addObject("Near_Hopper",near_hopper);
		SmartDashboard.putData("Select Auto",autoSelect);
		robot = RoboSystem.getInstance();
		controllers = TeleController.getInstance();
		fsm = FSM.getInstance();
		dist = DistanceController.getInstance();

/* SmartDashboard.putBoolean("Manual Wheel Headings?", true); SmartDashboard.putNumber("Manual Heading 1", 0);
 * SmartDashboard.putNumber("Manual Heading 2", 0); SmartDashboard.putNumber("Manual Heading 3", 0);
 * SmartDashboard.putNumber("Manual Heading 4", 0); /**/
	}

	public void autonomous() {
		String autoSelected = (String)autoSelect.getSelected();
		// robot.intake._pidgey.SetFusedHeading(180.0);
		// robot.dt.resetCoord();
		switch(autoSelected)
			{
				case one_gear:
					executeAuto(AUTO.ONE_GEAR);
					break;
				case two_gear:
					executeAuto(AUTO.TWO_GEAR);
					break;
				case near_hopper:
					executeAuto(AUTO.NEAR_HOPPER);
					break;
				case defaultAuto:
					executeAuto(AUTO.TWO_GEAR);
					break;
			}
	}

	/***
	 * Sets the gyro's Fused Heading to the given angle, then resets the drivetrain's coordinates.
	 * <p>
	 * The call to {@link com.ctre.PigeonImu#SetFusedHeading(double) SetFusedHeading} essentially ``zeroes'' the
	 * Pigeon's heading, but it does so at a given angle. For example, to tell the robot that the direction it is
	 * currently facing is now defined as 88 degrees, call {@code SetFusedHeading(88)}.
	 * </p>
	 * <p>
	 * The call to {@link SubSystems.Swerve#resetCoord() resetCoord} zeroes the robot's center coordinates and
	 * {@link SubSystems.Swerve.SwerveDriveModule#initModule() calculates} the appropriate initial coordinates of the
	 * encoder-bearing module(s). (In our case, the {@link SubSystems.Swerve#frontLeft front left} module, number 2,
	 * bears the lone drive encoder.) These coordinates are calculated using the robot's heading, so it is necessary to
	 * perform this step <em>after</em> setting the heading.
	 * </p>
	 * 
	 * @param angle The heading value to be assigned to the robot's current orientation
	 */
	private void initializeHeading(double angle) {
		robot.intake._pidgey.SetFusedHeading(angle);
		robot.dt.resetCoord();
// robot.dt.setHeading(angle);
	}

	public void rotate(double angle) {
		long timeout = System.currentTimeMillis() + 1000;
		robot.dt.setHeading(angle,true);
		while(!robot.dt.headingOnTarget() && isAutonomous() && (timeout > System.currentTimeMillis())) {
			robot.dt.sendInput(0,0,0,0,false,true,false);
			Timer.delay(0.01);
		}
	}

	private void driveTo(double x, double y, double err, double time, double power) {
		dist.setGoal(x,y,err,time,power);
		while(!dist.onTarget() && isAutonomous()) {Timer.delay(0.01);}
	}

	public void executeAuto(AUTO autoSelect) {
		switch(autoSelect)
			{
				case TWO_GEAR:
				//* This auto starts with the robot's front facing the driver station; id est, the robot is backward on the field.
					initializeHeading(180); /*/
					robot.intake._pidgey.SetFusedHeading(180.0); robot.dt.resetCoord(); /**/

				// TODO Adjust distances to be correct
				// TODO Tune timeouts/thresholds/coefficients, et cetera

				// The first gear sits atop the robot; we just need to get to the peg
					driveTo(0,57,0.2,1.5,0.8);// dist.setGoal(...);delay();
				// Move back to (near) starting position, where the second gear will be
					driveTo(0,30,0.2,1.2,0.8);
				// Rotate so that intake faces gear
					rotate(90);
				// Send instructions to intake in preparation to grab second gear
					/* move gear intake down */
					/* turn gear intake on */
				// Move forward just a hair to pick up second gear
					driveTo(-20,30,0.2,1.0,0.9);
				// Suck up gear, then do whatever we need to do with the intake
					/* turn gear intake off? */
					/* move gear intake up */
				// Move back to line ourselves up with the peg again
					driveTo(0,30,0.2,1.0,0.9);
				// Rotate so that gear intake faces peg
					rotate(180);
				// Move forward to peg
					driveTo(0,57,0.2,1.5,0.8);
				// Tell intake to deposit peg
					/* reverse gear intake to score */
					/* stop gear intake */
				// TODO Implement shooting
					break;

				case NEAR_HOPPER:
				//* This auto starts with the robot's front facing toward the boiler.
					initializeHeading(90);/*/
					robot.intake._pidgey.SetFusedHeading(90.0); robot.dt.resetCoord(); /**/

				// TODO Convince the robot to move in the correct directions
				// TODO Adjust distances to be correct
				// TODO Tune timeouts/thresholds/coefficients, et cetera

				// Move in a positive y direction towards the gear peg
					driveTo(0,50,2.0,5.0,0.7); // (0,102)
				// Move right just a bit to place the gear on the peg
					driveTo(20,50,2.0,5.0,0.7); // (15,102)
				// Move forward and reach the hopper, deploying it in the process
					driveTo(-10,50,2.0,5.0,0.7); // (-15,102)
				// Move in a positive x direction to fall in the path of the hopper's balls
					driveTo(-10,70,2.0,5.0,0.7); // (-15,110)
				// TODO Implement shooting
					break;
				case OFF:
					break;
				default:
					break;
			}
		while(isAutonomous()) {
			Timer.delay(1);
		}
	}

	public void operatorControl() {
		dist.disable();
	//	robot.dt.setHeading(0,false);
		while(isOperatorControl() && isEnabled()) {
			controllers.update();
			Timer.delay(0.01); // 10ms Loop Rate
		}
	}
}
