package com.team1323.steamworks2017;

import ControlSystem.FSM;
import ControlSystem.RoboSystem;
import Helpers.InterpolatingDouble;
import Helpers.Looper;
import IO.Logger;
import IO.TeleController;
import SubSystems.DistanceController;
import SubSystems.GearIntake;
import SubSystems.DistanceController.ControlWheel;
import SubSystems.Intake;
import SubSystems.Shooter;
import SubSystems.Swerve;
import SubSystems.Turret;
import SubSystems.VisionProcessor;
import SubSystems.VisionServer;
import Utilities.Constants;
import Utilities.Util;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Robot class is based on the {@link SampleRobot}.
 * */
public class Robot extends SampleRobot {
	private RoboSystem robot = RoboSystem.getInstance();
	private TeleController controllers;
	private FSM fsm = FSM.getInstance();
	private Logger logger;
	final String off = "off";
	final String one_gear    = "one_gear";
	final String gear_and_shoot = "gear_and_shoot";
	final String two_gear    = "two_gear";
	final String near_hopper = "near_hopper";
	final String far_hopper  = "far_hopper";
	final String near_hopper_inside = "near_hopper_inside";
	final String near_hopper_outside = "near_hopper_outside";
	final String near_hopper_receive = "near_hopper_receive";
	final String near_hopper_close_inside = "near_hopper_close_inside";
	final String blue = "Blue";
	final String red = "Red";
	long timeToWaitMs = 5500;
	public final int BLUE = 1;
	public final int RED  = -1;
	/** {@link SmartDashboard} field for selecting an autonomous subroutine to run */
	SendableChooser autoSelect;
	SendableChooser allianceSelect;
	
	/** {@link DistanceController} for sending autonomous or other pre-programmed
	 * waypoints to the {@link Swerve drivetrain} */
	private DistanceController dist;
	
	
	VisionServer mVisionServer = VisionServer.getInstance();
	/** Enabled looper is called at 100Hz whenever the robot is enabled */
    Looper mEnabledLooper = new Looper();
    /** Disabled looper is called at 100Hz whenever the robot is disabled */
    Looper mDisabledLooper = new Looper();
    
    
	public static enum AUTO{
    	/** Do not run any autonomous subroutine. */
		OFF,
    	/** Start at center position. Move field-forward to score gear. Shoot balls. */
		ONE_GEAR,
		GEAR_AND_SHOOT,
    	/** Start at center position. Move field-forward to score gear. Move field-backward and rotate to 90 or 270 degrees
    	 *   to face another gear on the field. Intake gear from floor. Rotate to 180 degrees and move field-forward to score
    	 *   gear. Shoot balls. */
    	/** Deploy hoppers on boiler side of field. Move under the outside hopper to receive balls. Shoot balls. */
		NEAR_HOPPER
    	/** Deploy hoppers on not-boiler side of field. Move under the outside hopper to receive balls. Move to field x-center,
    	 *    */
    }
	
    public Robot() {
    	autoSelect = new SendableChooser();
    	autoSelect.addDefault("Near_Hopper", near_hopper);
        autoSelect.addObject("Off", off);
        autoSelect.addObject("One_Gear", one_gear);
        autoSelect.addObject("Gear and Shoot", gear_and_shoot);
        
        
        SmartDashboard.putData(" Select Auto ", autoSelect);
        allianceSelect = new SendableChooser();
        allianceSelect.addDefault("Blue", blue);
        allianceSelect.addObject("Red", red);
        //allianceSelect.addObject("Blue", blue);
        
        SmartDashboard.putData(" Select  Alliance ", allianceSelect);
        
        robot = RoboSystem.getInstance();
        controllers = TeleController.getInstance();
        fsm = FSM.getInstance();
        dist = DistanceController.getInstance();
        robot.intake.setPresetAngles(0);
        Timer.delay(0.1);
        robot.dt.resetCoord(Swerve.AnglePresets.ZERO);
        Timer.delay(0.1);
    	/** Autonomous subroutine that the user has selected to run */
    	String autoSelected = (String) autoSelect.getSelected();
    	String allianceSelected = (String) allianceSelect.getSelected();
    	SmartDashboard.putString(" Auto Selected ", autoSelected);
    	SmartDashboard.putString(" Alliance Selected ", allianceSelected);
    	
    	logger = Logger.getInstance();
    	switch(allianceSelected){
    	case blue:
    		logger.writeToLog("TEAM BLUE");
    		robot.turret.resetAngle(90);
    		break;
    	case red:
    		logger.writeToLog("TEAM RED");
    		robot.turret.resetAngle(-90);
    		break;
    	default:
    		logger.writeToLog("TEAM DEFAULT (blue, Robot.java:149)");
    		robot.turret.resetAngle(90);
    		break;
    	}
    	robot.dt.disableReverse();
    }

	@Override
	public void robotInit() {
		mVisionServer.addVisionUpdateReceiver(VisionProcessor.getInstance());
		mEnabledLooper.register(VisionProcessor.getInstance());
		VisionServer.getInstance();
		mEnabledLooper.start();
	}
	public double getHeadingForRange(double range) {
        InterpolatingDouble result = Constants.kHeadingMap.getInterpolated(new InterpolatingDouble(range));
        if (result != null) {
            return result.value;
        } else {
            return 60.0;
        }
    }
    /** The autonomous routine, which calls the selected autonomous subroutine */
    public void autonomous() {
    	String autoSelected = (String) autoSelect.getSelected();
    	String allianceSelected = (String) allianceSelect.getSelected();
    	SmartDashboard.putString(" Auto Selected ", autoSelected);
    	SmartDashboard.putString(" Alliance Selected ", allianceSelected);
//    	robot.intake._pidgey.SetFusedHeading(180.0);
//    	robot.dt.resetCoord(AnglePresets.ONE_EIGHTY);
//    	robot.dt.setHeading(0.0);
    	//robot.dt.enableReverse();
    	int alliance = BLUE;
    	switch(allianceSelected){
	    	case blue:
	    		alliance = BLUE;
	    		robot.turret.resetAngle(90);
	    		break;
	    	case red:
	    		alliance = RED;
	    		robot.turret.resetAngle(-90);
	    		break;
	    	default:
	    		alliance = BLUE;
	    		robot.turret.resetAngle(90);
	    		break;
    	}
    		switch(autoSelected){
    			case one_gear:
	            	initHeading(0);
	            	executeAuto(AUTO.ONE_GEAR, alliance);
    				break;
    			case gear_and_shoot:
    				initHeading(0);
    				executeAuto(AUTO.GEAR_AND_SHOOT, alliance);
    				break;
    			case near_hopper:
    	            switch(allianceSelected){
	            	case blue:
	            		initHeading(180);
	    	            Timer.delay(0.1);
	            		executeAuto(AUTO.NEAR_HOPPER, BLUE);
	            		break;
	            	case red:
	            		initHeading(0);
	    	            Timer.delay(0.1);
	            		executeAuto(AUTO.NEAR_HOPPER, RED);
	            		break;
	            	default:
	            		initHeading(180);
	    	            Timer.delay(0.1);
	            		executeAuto(AUTO.NEAR_HOPPER, BLUE);
	            		break;
	            }
    				break;
    			case off:
    				executeAuto(AUTO.OFF, 1);
    				break;
    		}

    }
    private void delay2() {
		while(!dist.onTarget() && isAutonomous() && !dist.isOnTarget()){
			Timer.delay(0.005);
		}
    }
    private void delay() {
		while(dist.isEnabled() && isAutonomous()){
			Timer.delay(0.005);
		}
    }
    public void executeAuto(AUTO autoSelect, int team){
    	logger.writeToLog("executeAuto");
    	robot.dt.setAutoRampRate();
    	robot.dt.stopRotating();
    	robot.intake.deployWings();
    	robot.gearIntake.gearExtender();
    	robot.retractBallFlap();
    	long timeout = 0;
    	robot.dt.enableReverse();
    	robot.turret.stop();
    	// 1 is blue, -1 is red
    	switch(autoSelect){
    	case ONE_GEAR:
    		logger.writeToLog("AUTO One Gear");
    		/*dist.setYPID(Constants.DIST_CONTROLLER_Y_SHORT_P, 0.0, Constants.DIST_CONTROLLER_Y_SHORT_D, Constants.DIST_CONTROLLER_Y_SHORT_FF);
			dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, 12, ControlWheel.FOLLOWER, 0.0, 3.0, 0.8, 1, false);
			delay();*/
    		//67 to 77.5
    		//101 to 109
    		//offset from edge of carpet to gear intake is about 32
    		double offset = 32;
    		double edgeOfPeg = 101;
    		double wall = 109;
    		double distanceToPeg = edgeOfPeg - offset;
    		double distanceToWall = wall - offset;
    		logger.writeToLog("AUTO Gear and Shoot");
    		robot.gearIntake.setState(GearIntake.State.INTAKE_EXTENDED_OFF);
    		Timer.delay(0.25);
    		robot.gearIntake.retract();
    		dist.setXPID(0.005, 0.0, 0.0, 0.0);
    		dist.setYPID(Constants.DIST_CONTROLLER_Y_SHORT_P, 0.0, Constants.DIST_CONTROLLER_Y_SHORT_D, Constants.DIST_CONTROLLER_Y_SHORT_FF);
			dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, 16, ControlWheel.FOLLOWER, 2.0, 1.3, 0.8, 1, false);
			delay();
			dist.setYPID(Constants.DIST_CONTROLLER_Y_SHORT_P, 0.0, Constants.DIST_CONTROLLER_Y_SHORT_D, Constants.DIST_CONTROLLER_Y_SHORT_FF);
			dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, 6, ControlWheel.FOLLOWER, 3.0, 1.0, 0.8, 1, false);
			delay();
			robot.gearIntake.initiateAutoPickup();
			Timer.delay(0.15);
    		dist.setYPID(0.015, 0.0, 0.01, Constants.DIST_CONTROLLER_Y_LONG_FF);
    		dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, distanceToPeg - 4, ControlWheel.FOLLOWER, 2.0, 2.5, 0.6, 3, false);
    		//robot.turret.setState(Turret.State.VisionTracking);
    		delay();
    		timeout = System.currentTimeMillis() + 1750;
    		robot.dt.setHeading(0, true);
    		while(isAutonomous() && timeout > System.currentTimeMillis() && dist.getYInches() < distanceToWall){
    			robot.dt.sendInput(0, 0.3, 0, 0, false, false, false, false);
    			Timer.delay(0.01);
    		}
    		System.out.println("Y before completion check: " + Double.toString(dist.getYInches()));
    		if(distanceToWall - dist.getYInches() > 3.0){
    			dist.setYPID(Constants.DIST_CONTROLLER_Y_SHORT_P, 0.0, Constants.DIST_CONTROLLER_Y_SHORT_D, Constants.DIST_CONTROLLER_Y_SHORT_FF);
    			dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, dist.getYInches() - 8, ControlWheel.FOLLOWER, 2.0, 1.0, 0.4, 10, false);
    			delay();
    			System.out.println("Heading before 4 degrees: " + Double.toString(robot.intake.getCurrentAngle()));
    			robot.dt.setHeading(robot.intake.getCurrentAngle() - 7, true);
    			System.out.println("Heading after correction: " + Double.toString(robot.intake.getCurrentAngle()));
    			timeout = System.currentTimeMillis() + 1500;
    			while(isAutonomous() && timeout > System.currentTimeMillis() && dist.getYInches() < distanceToWall){
        			robot.dt.sendInput(0.0, 0.3, 0, 0, false, false, false, false);
        			Timer.delay(0.01);
        		}
    			System.out.println("Y after correction: " + Double.toString(dist.getYInches()));
    		}
    		if(distanceToWall - dist.getYInches() > 3.0){
    			dist.setYPID(Constants.DIST_CONTROLLER_Y_SHORT_P, 0.0, Constants.DIST_CONTROLLER_Y_SHORT_D, Constants.DIST_CONTROLLER_Y_SHORT_FF);
    			dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, dist.getYInches() - 8, ControlWheel.FOLLOWER, 2.0, 1.0, 0.4, 10, false);
    			delay();
    			System.out.println("Heading before 4 degrees: " + Double.toString(robot.intake.getCurrentAngle()));
    			robot.dt.setHeading(robot.intake.getCurrentAngle() - 7, true);
    			System.out.println("Heading before 4 degrees: " + Double.toString(robot.intake.getCurrentAngle()));
    			timeout = System.currentTimeMillis() + 1500;
    			while(isAutonomous() && timeout > System.currentTimeMillis() && dist.getYInches() < distanceToWall){
        			robot.dt.sendInput(0.0, 0.3, 0, 0, false, false, false, false);
        			Timer.delay(0.01);
        		}
    			System.out.println("Y after correction: " + Double.toString(dist.getYInches()));
    		}
    		robot.gearIntake.scoreGear();
    		Timer.delay(0.4);
    		dist.setYPID(Constants.DIST_CONTROLLER_Y_SHORT_P, 0.0, Constants.DIST_CONTROLLER_Y_SHORT_D, Constants.DIST_CONTROLLER_Y_SHORT_FF);
			dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, distanceToPeg - 2, ControlWheel.FOLLOWER, 2.0, 1.0, 0.5, 5, false);
			delay();
			robot.gearIntake.retract();
			dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, distanceToWall, ControlWheel.FOLLOWER, 2.0, 1.2, 0.6, 5, false);
			delay();
    		dist.setYPID(Constants.DIST_CONTROLLER_Y_LONG_P, 0.0, Constants.DIST_CONTROLLER_Y_LONG_D, Constants.DIST_CONTROLLER_Y_LONG_FF);
    		dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, dist.getYInches() - 50, ControlWheel.FOLLOWER, 2.0, 2.5, 0.5, 10, false);
    		delay();
    		break;
    	case GEAR_AND_SHOOT:
    		double sideHopperY = 130;
    		double sideOffset = 40;
    		double robotDistanceFirst = sideHopperY - sideOffset;
    		double degreesToPeg = 60.0;
    		double distanceToScore = 30.0;
    		double distanceToEdge = distanceToScore - 8;
    		robot.gearIntake.setState(GearIntake.State.INTAKE_EXTENDED_OFF);
    		Timer.delay(0.25);
    		robot.gearIntake.retract();
    		
    		
    		robot.shooter.setGoal(Constants.SHOOTING_SPEED);
    		robot.shooter.setState(Shooter.Status.STARTED);
    		
    		dist.setXPID(0.005, 0.0, 0.0, 0.0);
    		dist.setYPID(Constants.DIST_CONTROLLER_Y_SHORT_P, 0.0, Constants.DIST_CONTROLLER_Y_SHORT_D, Constants.DIST_CONTROLLER_Y_SHORT_FF);
			dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, 16, ControlWheel.FOLLOWER, 2.0, 1.3, 0.8, 1, false);
			delay();
			robot.turret.lockAngle(robot.intake.getCurrentAngle(),-7*team);
    		robot.turret.setState(Turret.State.GyroComp);
			dist.setYPID(Constants.DIST_CONTROLLER_Y_SHORT_P, 0.0, Constants.DIST_CONTROLLER_Y_SHORT_D, Constants.DIST_CONTROLLER_Y_SHORT_FF);
			dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, 2, ControlWheel.FOLLOWER, 3.0, 1.0, 0.8, 1, false);
			delay();
			System.out.println("Follower Y for turret calculation: " + Double.toString(dist.getYInches()));
			System.out.println("Robot heading for turret Adjustment: " + Double.toString(robot.intake.getCurrentAngle()));
			System.out.println("Calculated Turret angle: " + Double.toString(robot.turret.getTurretAngleForRange(dist.getYInches())));
			robot.turret.lockAngle(-360.0, robot.turret.getTurretAngleForRange(dist.getYInches()));
    		while (robot.shooter.getStatus()!=Shooter.Status.READY && isAutonomous()){
    			Timer.delay(0.01);
    		}
    		robot.sweeper.turnSweeperOn();
    		timeout = System.currentTimeMillis() + 1000;
    		while(System.currentTimeMillis() < timeout && isAutonomous()){
    			Timer.delay(0.01);
    		}
			//robot.gearIntake.initiateAutoPickup();
			Timer.delay(0.15);
			robot.turret.setState(Turret.State.Off);
			robot.sweeper.stopRoller();
    		robot.sweeper.stopSweeper();
    		robot.intake.intakeStop();
    		robot.shooter.setState(Shooter.Status.OFF);
    		robot.gearIntake.initiateAutoPickup();
			Timer.delay(0.15);
    		dist.setYPID(0.014, 0.0, 0.01, Constants.DIST_CONTROLLER_Y_LONG_FF);
    		dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, robotDistanceFirst, ControlWheel.FOLLOWER, 2.0, 2.75, 0.7, 3, false);
    		delay();
    		dist.disable();
    		double firstDistReached = dist.getYInches();
    		int correctDirection = 1;
    		if(firstDistReached < robotDistanceFirst){
    			correctDirection = 1;
    		}else{
    			correctDirection = -1;
    		}
    		degreesToPeg = getHeadingForRange(robotDistanceFirst - dist.getYInches());
    		System.out.println("Calculated Peg Rotation: " + Double.toString(degreesToPeg) + " Distance Reached: " + Double.toString(dist.getYInches()));
    		robot.dt.setHeading(degreesToPeg, true);
    		robot.dt.enableRotation();
    		timeout = System.currentTimeMillis() + 3000;
    		while(isAutonomous() && !robot.dt.headingOnTarget() && System.currentTimeMillis() < timeout){
    			robot.dt.sendInput(0.0, 0.0, 0.0, 0.0, false, false, false, false);
    			Timer.delay(0.01);
    		}
    		robot.dt.disableRotation();
    		double targetY = dist.getYInches() + distanceToScore;
    		System.out.println("Calculated Wall Distance: " + Double.toString(targetY));
    		timeout = System.currentTimeMillis() + 2500;
    		while(isAutonomous() && System.currentTimeMillis() < timeout && dist.getYInches() < targetY){
    			robot.dt.sendInput(0, 0.35, 0, 0, false, true, false, false);
    			Timer.delay(0.01);
    		}
    		System.out.println("Y before completion check: " + Double.toString(dist.getYInches()));
    		if(targetY - dist.getYInches() > 2.0){
    			dist.setYPID(Constants.DIST_CONTROLLER_Y_SHORT_P, 0.0, Constants.DIST_CONTROLLER_Y_SHORT_D, Constants.DIST_CONTROLLER_Y_SHORT_FF);
    			dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, dist.getYInches() - 8, ControlWheel.FOLLOWER, 2.0, 1.0, 0.4, 10, false, true);
    			delay();
    			System.out.println("Heading before 4 degrees: " + Double.toString(robot.intake.getCurrentAngle()));
    			robot.dt.setHeading(robot.intake.getCurrentAngle() - (7*correctDirection), true);
    			System.out.println("Heading after correction: " + Double.toString(robot.intake.getCurrentAngle()));
    			timeout = System.currentTimeMillis() + 1500;
    			while(isAutonomous() && timeout > System.currentTimeMillis() && dist.getYInches() < targetY){
        			robot.dt.sendInput(0.0, 0.3, 0, 0, false, true, false, false);
        			Timer.delay(0.01);
        		}
    			System.out.println("Y after correction: " + Double.toString(dist.getYInches()));
    		}
    		if(targetY - dist.getYInches() > 2.0){
    			dist.setYPID(Constants.DIST_CONTROLLER_Y_SHORT_P, 0.0, Constants.DIST_CONTROLLER_Y_SHORT_D, Constants.DIST_CONTROLLER_Y_SHORT_FF);
    			dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, dist.getYInches() - 8, ControlWheel.FOLLOWER, 2.0, 1.0, 0.4, 10, false, true);
    			delay();
    			System.out.println("Heading before 4 degrees: " + Double.toString(robot.intake.getCurrentAngle()));
    			robot.dt.setHeading(robot.intake.getCurrentAngle() - (7*correctDirection), true);
    			System.out.println("Heading before 4 degrees: " + Double.toString(robot.intake.getCurrentAngle()));
    			timeout = System.currentTimeMillis() + 1500;
    			while(isAutonomous() && timeout > System.currentTimeMillis() && dist.getYInches() < targetY){
        			robot.dt.sendInput(0.0, 0.3, 0, 0, false, true, false, false);
        			Timer.delay(0.01);
        		}
    			System.out.println("Y after correction: " + Double.toString(dist.getYInches()));
    		}
    		robot.gearIntake.scoreGear();
    		Timer.delay(0.4);
    		dist.setYPID(Constants.DIST_CONTROLLER_Y_SHORT_P, 0.0, Constants.DIST_CONTROLLER_Y_SHORT_D, Constants.DIST_CONTROLLER_Y_SHORT_FF);
			dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, targetY - 10, ControlWheel.FOLLOWER, 2.0, 1.0, 0.5, 5, false, true);
			delay();
			robot.gearIntake.retract();
			dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, targetY, ControlWheel.FOLLOWER, 2.0, 1.2, 0.6, 5, false, true);
			delay();
    		dist.setYPID(Constants.DIST_CONTROLLER_Y_LONG_P, 0.0, Constants.DIST_CONTROLLER_Y_LONG_D, Constants.DIST_CONTROLLER_Y_LONG_FF);
    		dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, dist.getYInches() - 35, ControlWheel.FOLLOWER, 2.0, 2.5, 0.5, 10, false, true);
    		delay();
    		break;
    	case NEAR_HOPPER:
    		
    		//Step 1
    		double closeHopperY = 79.0;
    		dist.setXPID(0.005, 0.0, 0.0, 0.0);
    		dist.setYPID(0.011, 0.0, Constants.DIST_CONTROLLER_Y_LONG_D, Constants.DIST_CONTROLLER_Y_LONG_FF);
    		logger.writeToLog("AUTO Near Hopper Close Inside");
    		logger.writeToLog("Y distance before: " + Double.toString(robot.dt.frontRight.getNegatedFollowerWheelInches()) + " X distance: " + Double.toString(robot.dt.getX()));
    		dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, closeHopperY, ControlWheel.FOLLOWER, 2.5, 2.55, .85, 5, true); //2.3 Timeout
//    		robot.turret.setAngle(103);
    		robot.turret.lockAngle(robot.intake.getCurrentAngle(),98*team);
    		robot.turret.setState(Turret.State.GyroComp);
    		switch(team){
	    		case BLUE:
	//    			robot.dt.setHeading(180, true);
	    			break;
	    		case RED:
	//    			robot.dt.setHeading(0, true);
	    			break;
    		}
    		
    		//robot.turret.setState(Turret.State.VisionTracking);
    		//robot.deployBallFlap();
    		delay();
    		//delay2();
    		//dist.disable();
    		logger.writeToLog("Y distance: " + Double.toString(robot.dt.frontRight.getNegatedFollowerWheelInches()));
    		logger.writeToLog("Vision Distance: " + Double.toString(fsm.getTargetDistance()));
    		
    		
    		//Step 2 
    		//Add Timeout 
    		dist.setXPID(Constants.DIST_CONTROLLER_X_P, 0.0, Constants.DIST_CONTROLLER_X_D, 0.0);
    		//dist.setYPID(Constants.DIST_CONTROLLER_Y_SHORT_P, 0.0, Constants.DIST_CONTROLLER_Y_SHORT_D, Constants.DIST_CONTROLLER_Y_SHORT_FF);
    		dist.setGoal(robot.dt.getX()+((-36)*team), ControlWheel.SWERVE, closeHopperY, ControlWheel.FOLLOWER, 2.0, 1.25, 0.6, 0, true);
    		long time = System.currentTimeMillis();
    		timeout = System.currentTimeMillis() + 750;
    		int i = 1;
    		while(Math.abs(robot.dt.getX()) < 12 && isAutonomous() && System.currentTimeMillis() < timeout){
    			if(i < 10){
    				System.out.println("Inside 12 inch while loop" + Integer.toString(i));
    			}
    			i++;
    			Timer.delay(0.01);
    		}
    		timeout = System.currentTimeMillis() + (1600 - (System.currentTimeMillis() - time));
    		System.out.println("Line 321 reached");
    		i = 1;
    		while(isAutonomous() && System.currentTimeMillis() < timeout && robot.intake.getCurrentAngularRate() < 35){
    			if(i < 5){
    				System.out.println("Inside Pigeon Loop " + Integer.toString(i));
    			}
    			i++;
    			Timer.delay(0.005);
    		}
    		System.out.println("Line 328 reached");
    		dist.disable();
    		//delay();
    	
    		logger.writeToLog("AUTO POS1 X:" + Double.toString(dist.getXInches()) + " Y:"+ Double.toString(dist.getYInches()));
    		logger.writeToLog("Vision Distance: " + Double.toString(fsm.getTargetDistance()));
    		
    		//Step 3
    		//Add swerve + turret lock and then feed vision as a correction
    		dist.setXPID(0.005, 0.0, 0.0, 0.0);
    		dist.setYPID(Constants.DIST_CONTROLLER_Y_SHORT_P, 0.0, Constants.DIST_CONTROLLER_Y_SHORT_D, 0.22);
    		dist.setGoal(robot.dt.getX() + 3, ControlWheel.SWERVE, 70, ControlWheel.FOLLOWER, 2.0, 1.75, 1.0, 10, true); 	
    		
    		if(fsm.getTargetVisibility()){
    			robot.shooter.setGoal(robot.shooter.getShooterSpeedForRange(fsm.getTargetDistance()-3));
    		}else{
    			robot.shooter.setGoal(Constants.SHOOTING_SPEED - 200);
    		}
    		robot.shooter.setState(Shooter.Status.STARTED);
    		
    		delay();
    		robot.deployBallFlap();
    		//dist.disable();
    		robot.shooter.setGoal(robot.shooter.getShooterSpeedForRange(dist.getYInches()));
    		System.out.println("Shooter Target: " + Double.toString(robot.shooter.getTarget()));
    		robot.shooter.setState(Shooter.Status.STARTED);
    		robot.dt.sendInput(-0.2*team, 0, 0, 0, false, false, false, false);
    		if(!fsm.getTargetVisibility()){
    			if(team == BLUE){
    				logger.writeToLog("Robot Heading: " + Double.toString(robot.intake.getCurrentAngle()));
    				//robot.turret.setAngle(103+(180-Util.boundAngle0to360Degrees(robot.intake.getCurrentAngle())));
    			}else{
    				//robot.turret.setAngle(-75+(180-Util.boundAngle0to360Degrees(robot.intake.getCurrentAngle())));
    			}
    		}
    		
    		//Step 4
    		logger.writeToLog("Distance Y:" + dist.getYInches());
    		logger.writeToLog("Robot Heading: " + Double.toString(robot.intake.getCurrentAngle()));
    		logger.writeToLog("Turret Angle: " + Double.toString(robot.turret.getAngle()));
    		
    		//robot.turret.lockAngle(robot.intake.getCurrentAngle());
    		//robot.turret.setState(Turret.State.GyroComp);
    		
    		//robot.shooter.setGoal(Constants.SHOOTING_SPEED);
    		logger.writeToLog("Vision Distance Before Shooter: " + Double.toString(fsm.getTargetDistance()));
    		
    		robot.intake.intakeForward();
    		while (robot.shooter.getStatus()!=Shooter.Status.READY && isAutonomous()){
    			Timer.delay(0.01);
    		}
    		robot.sweeper.turnSweeperOn();
    		
    		while(isAutonomous()){
    			Timer.delay(0.01);
    			
    		}
    		robot.dt.sendInput(0, 0, 0, 0, false, false, false, false);
    		robot.sweeper.stopRoller();
    		robot.sweeper.stopSweeper();
    		robot.intake.intakeStop();
    		robot.shooter.setState(Shooter.Status.OFF);
    		break;
    	case OFF:
    		logger.writeToLog("AUTO *** OFF ***");
    		break;
    	default: 
    		logger.writeToLog("AUTO *** DEFAULT ***");
    		break;
    }
    	while(isAutonomous())
    	{Timer.delay(0.1);}
    }
    
    private void initHeading(double angle){
    	robot.intake.setPresetAngles(angle);
        Timer.delay(0.1);
        robot.dt.resetCoord(Swerve.AnglePresets.ZERO);
        Timer.delay(0.1);
    }
    
    public void operatorControl() {
    	robot.dt.disableReverse();
    	robot.dt.setTeleRampRate();
    	robot.dt.stopRotating();
    	robot.turret.stop();
//    	robot.dt.enableReverse();
    	dist.disable();
    	robot.shooter.stop();
    	robot.sweeper.stopSweeper();
    	robot.sweeper.stopRoller();
    	robot.dt.setHeading(robot.intake.getCurrentAngle(), false);
        while (isOperatorControl() && isEnabled()) {        	
        	controllers.update();
            Timer.delay(0.01);		//10ms Loop Rate
        }
    }
}