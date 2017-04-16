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

public class Robot extends SampleRobot {
	private RoboSystem robot = RoboSystem.getInstance();
	private TeleController controllers;
	private FSM fsm = FSM.getInstance();
	private Logger logger;
	final String off = "off";
	final String one_gear    = "one_gear";
	final String gear_and_shoot = "gear_and_shoot";
	final String near_hopper = "near_hopper";
	final String blue = "Blue";
	final String red = "Red";
	public final int BLUE = 1;
	public final int RED  = -1;
	public int alliance = BLUE;
	SendableChooser autoSelect;
	SendableChooser allianceSelect;
	
	private DistanceController dist;
	
	
	VisionServer mVisionServer = VisionServer.getInstance();
    Looper mEnabledLooper = new Looper();
    Looper mDisabledLooper = new Looper();
    
	public static enum AUTO{
		OFF,
		ONE_GEAR,
		GEAR_AND_SHOOT,
		NEAR_HOPPER
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
		robot.initCamera();
		mEnabledLooper.start();
	}
	@Override
	public void disabled(){
		robot.turret.setState(Turret.State.Off);
		robot.turret.stop();
	}
	public double getHeadingForRange(double range) {
        InterpolatingDouble result = Constants.kHeadingMap.getInterpolated(new InterpolatingDouble(range));
        if (result != null) {
            return result.value;
        } else {
            return 60.0;
        }
    }
	public double getDistanceToWallCorrection(double range) {
        InterpolatingDouble result = Constants.kDistanceToWallMap.getInterpolated(new InterpolatingDouble(range));
        if (result != null) {
            return result.value;
        } else {
            return 0.0;
        }
    }
	public double getUltrasonicCorrection(double range) {
        InterpolatingDouble result = Constants.kDistanceToWallUltraMap.getInterpolated(new InterpolatingDouble(range));
        if (result != null) {
            return result.value;
        } else {
            return 35.0;
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
    	alliance = BLUE;
    	switch(allianceSelected){
	    	case blue:
	    		alliance = BLUE;
	    		robot.setColor(BLUE);
	    		robot.turret.resetAngle(90);
	    		break;
	    	case red:
	    		alliance = RED;
	    		robot.setColor(RED);
	    		robot.turret.resetAngle(-90);
	    		break;
	    	default:
	    		alliance = BLUE;
	    		robot.setColor(BLUE);
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
	    	            Timer.delay(0.15);
	            		executeAuto(AUTO.NEAR_HOPPER, BLUE);
	            		break;
	            	case red:
	            		initHeading(0);
	    	            Timer.delay(0.15);
	            		executeAuto(AUTO.NEAR_HOPPER, RED);
	            		break;
	            	default:
	            		initHeading(180);
	    	            Timer.delay(0.15);
	            		executeAuto(AUTO.NEAR_HOPPER, BLUE);
	            		break;
	            }
    				break;
    			case off:
    				switch(allianceSelected){
	            	case blue:
	            		initHeading(180);
	    	            Timer.delay(0.1);
	            		executeAuto(AUTO.OFF, BLUE);
	            		break;
	            	case red:
	            		initHeading(0);
	    	            Timer.delay(0.1);
	            		executeAuto(AUTO.OFF, RED);
	            		break;
	            	default:
	            		initHeading(180);
	    	            Timer.delay(0.1);
	            		executeAuto(AUTO.OFF, BLUE);
	            		break;
	            }
    				break;
    		}

    }
    
    private void delay() {
		while(dist.isEnabled() && isAutonomous()){
			Timer.delay(0.01); //.005
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
    	//robot.turret.stop();
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
			robot.dt.setHeading(0, true);
			Timer.delay(0.15);
    		dist.setYPID(0.015, 0.0, 0.01, Constants.DIST_CONTROLLER_Y_LONG_FF);
    		dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, distanceToPeg - 4, ControlWheel.FOLLOWER, 2.0, 2.5, 0.6, 3, false);
    		robot.dt.setHeading(0, true);
    		delay();
    		timeout = System.currentTimeMillis() + 1750;
    		robot.dt.setHeading(0, true);
    		while(isAutonomous() && timeout > System.currentTimeMillis() && dist.getYInches() < distanceToWall){
    			robot.dt.sendInput(0, 0.3, 0, 0, false, false, false, false);
    			Timer.delay(0.01);
    		}
    		double correctionDirection = 1.0;
    		if(Util.boundAngle0to360Degrees(robot.intake.getCurrentAngle()) > 0){
    			correctionDirection = 1.0;
    		}else{
    			correctionDirection = -1.0;
    		}
    		System.out.println("Y before completion check: " + Double.toString(dist.getYInches()));
    		if(distanceToWall - dist.getYInches() > 1.5){
    			dist.setYPID(Constants.DIST_CONTROLLER_Y_SHORT_P, 0.0, Constants.DIST_CONTROLLER_Y_SHORT_D, Constants.DIST_CONTROLLER_Y_SHORT_FF);
    			dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, dist.getYInches() - 8, ControlWheel.FOLLOWER, 2.0, 1.0, 0.4, 10, false);
    			delay();
    			System.out.println("Heading before 4 degrees: " + Double.toString(robot.intake.getCurrentAngle()));
    			robot.dt.setHeading(robot.intake.getCurrentAngle() - (7*correctionDirection), true);
    			System.out.println("Heading after correction: " + Double.toString(robot.intake.getCurrentAngle()));
    			timeout = System.currentTimeMillis() + 1500;
    			while(isAutonomous() && timeout > System.currentTimeMillis() && dist.getYInches() < distanceToWall){
        			robot.dt.sendInput(0.0, 0.3, 0, 0, false, false, false, false);
        			Timer.delay(0.01);
        		}
    			System.out.println("Y after correction: " + Double.toString(dist.getYInches()));
    		}
    		if(distanceToWall - dist.getYInches() > 1.5){
    			dist.setYPID(Constants.DIST_CONTROLLER_Y_SHORT_P, 0.0, Constants.DIST_CONTROLLER_Y_SHORT_D, Constants.DIST_CONTROLLER_Y_SHORT_FF);
    			dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, dist.getYInches() - 8, ControlWheel.FOLLOWER, 2.0, 1.0, 0.4, 10, false);
    			delay();
    			System.out.println("Heading before 4 degrees: " + Double.toString(robot.intake.getCurrentAngle()));
    			robot.dt.setHeading(robot.intake.getCurrentAngle() - (14*correctionDirection*(-1)), true);
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
    		double distanceToScore = 27.0;
    		double distanceToEdge = distanceToScore - 8;
    		double ultraScoreDistance = 38.0;
    		robot.gearIntake.setState(GearIntake.State.INTAKE_EXTENDED_OFF);
    		Timer.delay(0.25);
    		robot.gearIntake.retract();
    		
    		
    		robot.shooter.setGoal(Constants.SIDE_PEG_SHOOTING_SPEED);
    		robot.shooter.setState(Shooter.Status.STARTED);
    		
    		dist.setXPID(0.005, 0.0, 0.0, 0.0);
    		dist.setYPID(Constants.DIST_CONTROLLER_Y_SHORT_P, 0.0, Constants.DIST_CONTROLLER_Y_SHORT_D, Constants.DIST_CONTROLLER_Y_SHORT_FF);
			dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, 16, ControlWheel.FOLLOWER, 2.0, 1.3, 0.8, 1, false);
			delay();
			if(team == BLUE){
				robot.turret.lockAngle(robot.intake.getCurrentAngle(),-5);
			}else{
				robot.turret.lockAngle(robot.intake.getCurrentAngle(),185);
			}
    		robot.turret.setState(Turret.State.GyroComp);
			dist.setYPID(Constants.DIST_CONTROLLER_Y_SHORT_P, 0.0, Constants.DIST_CONTROLLER_Y_SHORT_D, Constants.DIST_CONTROLLER_Y_SHORT_FF);
			dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, 2, ControlWheel.FOLLOWER, 3.0, 1.0, 0.8, 1, false);
			delay();
			System.out.println("Follower Y for turret calculation: " + Double.toString(dist.getYInches()));
			System.out.println("Robot heading for turret Adjustment: " + Double.toString(robot.intake.getCurrentAngle()));
			System.out.println("Calculated Turret correction: " + Double.toString(robot.turret.getTurretAngleForRange(dist.getYInches())));
			dist.disable();
			robot.turret.lockAngle(-360.0,-5 + robot.turret.getTurretAngleForRange(dist.getYInches()));
			timeout = System.currentTimeMillis() + 750;
    		while (robot.shooter.getStatus()!=Shooter.Status.READY && isAutonomous() && System.currentTimeMillis() < timeout){
    			Timer.delay(0.01);
    		}
    		timeout = System.currentTimeMillis() + 750; 
    		while(!robot.turret.onTarget() && isAutonomous() && System.currentTimeMillis() < timeout){
    			Timer.delay(0.01);
    		}
    		System.out.println("Final Turret Angle: " + Double.toString(robot.turret.getAngle()));
    		robot.sweeper.turnSweeperOn();
    		timeout = System.currentTimeMillis() + 1000;
    		while(System.currentTimeMillis() < timeout && isAutonomous()){
    			Timer.delay(0.01);
    		}
    		robot.gearIntake.initiateAutoPickup();
			robot.turret.setState(Turret.State.Off);
			robot.sweeper.stopRoller();
    		robot.sweeper.stopSweeper();
    		robot.intake.intakeStop();
    		robot.shooter.setState(Shooter.Status.OFF);
    		//dist.setYPID(0.012, 0.0, Constants.DIST_CONTROLLER_Y_LONG_D, Constants.DIST_CONTROLLER_Y_LONG_FF);
    		dist.setYPID(0.008, 0.0, 0.04, 0.10);
    		robot.dt.setHeading(0, true);
    		dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, robotDistanceFirst, ControlWheel.FOLLOWER, 2.0, /*2.75*/3.0, 0.85, 3, false);
    		delay();
    		dist.disable();
    		double firstDistReached = dist.getYInches();
    		double error = robotDistanceFirst - dist.getYInches();
    		int correctDirection = 1;
    		if(firstDistReached < robotDistanceFirst){
    			correctDirection = -1;
    		}else{
    			correctDirection = 1;
    		}
    		degreesToPeg = getHeadingForRange(error);
    		System.out.println("Calculated Peg Rotation: " + Double.toString(degreesToPeg) + " Distance Reached: " + Double.toString(dist.getYInches()));
    		robot.dt.setHeading(degreesToPeg*team, true);
    		robot.dt.enableRotation();
    		timeout = System.currentTimeMillis() + 3000;
    		while(isAutonomous() && !robot.dt.headingOnTarget() && System.currentTimeMillis() < timeout){
    			robot.dt.sendInput(0.0, 0.0, 0.0, 0.0, false, false, false, false);
    			Timer.delay(0.01);
    		}
    		robot.dt.disableRotation();
    		if(robot.dt.getError() > 0){
    			correctDirection = 1;
    		}else{
    			correctDirection = -1;
    		}
    		System.out.println("Ultra distance: " + Double.toString(robot.getDistance()));
    		System.out.println("Calculated Distance To WALL Correction: " + getDistanceToWallCorrection(error));
    		double targetY = dist.getYInches() + distanceToScore + getDistanceToWallCorrection(error);
    		System.out.println("Calculated Wall Distance: " + Double.toString(targetY));
    		timeout = System.currentTimeMillis() + 2500;
    		double ultraDistance = getUltrasonicCorrection(robot.intake.getCurrentAngle());
    		System.out.println("Ultra goal distance3: " + Double.toString(ultraDistance));
    		System.out.println("actual heading: " + Double.toString(robot.intake.getCurrentAngle()));
    		while(isAutonomous() && System.currentTimeMillis() < timeout && dist.getYInches() < targetY){
    			robot.dt.sendInput(0, 0.35, 0, 0, false, true, false, false);        		
    			Timer.delay(0.01);
    		}
    		robot.dt.sendInput(0, 0.0, 0, 0, false, true, false, false);
    		//Timer.delay(0.5);
    		System.out.println("Ultra goal distance: " + Double.toString(ultraDistance) + " Rob Angle: " + robot.intake.getCurrentAngle());
    		System.out.println("Y before completion check: " + Double.toString(dist.getYInches()) + " Ultra: " + Double.toString(robot.getDistance()));
    		if(targetY - dist.getYInches() > 0.5/*robot.getDistance() > ultraDistance*/){
    			dist.setYPID(Constants.DIST_CONTROLLER_Y_SHORT_P, 0.0, Constants.DIST_CONTROLLER_Y_SHORT_D, Constants.DIST_CONTROLLER_Y_SHORT_FF);
    			dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, dist.getYInches() - 8, ControlWheel.FOLLOWER, 2.0, 1.0, 0.4, 10, false, true);
    			delay();
    			System.out.println("Heading before 4 degrees: " + Double.toString(robot.intake.getCurrentAngle()));
    			robot.dt.setHeading(robot.intake.getCurrentAngle() + (8*correctDirection), true);
    			System.out.println("Heading after correction: " + Double.toString(robot.intake.getCurrentAngle()));
    			timeout = System.currentTimeMillis() + 1750;
    			while(isAutonomous() && timeout > System.currentTimeMillis() && dist.getYInches() < targetY){
        			robot.dt.sendInput(0.0, 0.3, 0, 0, false, true, false, false);
        			Timer.delay(0.01);
        		}
    			robot.dt.sendInput(0.0, 0.0, 0, 0, false, true, false, false);
    			System.out.println("Y after correction: " + Double.toString(dist.getYInches()) + " Ultra: " + Double.toString(robot.getDistance()));
    		}
    		ultraDistance = getUltrasonicCorrection(robot.intake.getCurrentAngle());
    		System.out.println("New ultrasonic goal: " + Double.toString(ultraDistance));
    		if(targetY - dist.getYInches() > 0.5/*robot.getDistance() > ultraDistance*/){
    			System.out.println("Ultrasonic: " + Double.toString(robot.getDistance()));
    			dist.setYPID(Constants.DIST_CONTROLLER_Y_SHORT_P, 0.0, Constants.DIST_CONTROLLER_Y_SHORT_D, Constants.DIST_CONTROLLER_Y_SHORT_FF);
    			dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, dist.getYInches() - 8, ControlWheel.FOLLOWER, 2.0, 1.0, 0.4, 10, false, true);
    			delay();
    			System.out.println("Heading before 4 degrees: " + Double.toString(robot.intake.getCurrentAngle()));
    			robot.dt.setHeading(robot.intake.getCurrentAngle() + (14*correctDirection*(-1)), true);
    			System.out.println("Heading before 4 degrees: " + Double.toString(robot.intake.getCurrentAngle()));
    			timeout = System.currentTimeMillis() + 1750;
    			while(isAutonomous() && timeout > System.currentTimeMillis() && dist.getYInches() < targetY){
        			robot.dt.sendInput(0.0, 0.3, 0, 0, false, true, false, false);
        			Timer.delay(0.01);
        		}
    			System.out.println("Y after correction: " + Double.toString(dist.getYInches()) + " Ultrasonic: " + Double.toString(robot.getDistance()));
    			robot.dt.sendInput(0.0, 0.0, 0, 0, false, true, false, false);
    		}
    		robot.gearIntake.scoreGear();
    		/*dist.setYPID(Constants.DIST_CONTROLLER_Y_SHORT_P, 0.0, Constants.DIST_CONTROLLER_Y_SHORT_D, Constants.DIST_CONTROLLER_Y_SHORT_FF);
			dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, targetY - 10, ControlWheel.FOLLOWER, 2.0, 1.0, 0.5, 5, false, true);
			delay();
			robot.gearIntake.firePiston();
			Timer.delay(0.25);
			robot.gearIntake.setState(GearIntake.State.INTAKE_RETRACTED);
			dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, targetY, ControlWheel.FOLLOWER, 2.0, 1.2, 0.6, 5, false, true);
			delay();*/
    		dist.setYPID(Constants.DIST_CONTROLLER_Y_LONG_P, 0.0, Constants.DIST_CONTROLLER_Y_LONG_D, Constants.DIST_CONTROLLER_Y_LONG_FF);
    		dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, dist.getYInches() - 35, ControlWheel.FOLLOWER, 2.0, 2.5, 0.5, 10, false, true);
    		delay();
    		break;
    	case NEAR_HOPPER:
    		robot.dt.zeroWheels();
    		System.out.println("Starting Turret Angle: " + Double.toString(robot.turret.getAngle()));
    		//Step 1
    		double closeHopperY = 82.0;
    		dist.setXPID(0.005, 0.0, 0.0, 0.0);
    		dist.setYPID(0.009, 0.0, 0.03, 0.1);
    		//logger.writeToLog("AUTO Near Hopper Close Inside");
    		logger.writeToLog("Y distance before: " + Double.toString(robot.dt.frontRight.getNegatedFollowerWheelInches()) + " X distance: " + Double.toString(robot.dt.getX()));
    		if(team == BLUE){
    			dist.setGoal(robot.dt.getX()-1, ControlWheel.SWERVE, closeHopperY, ControlWheel.FOLLOWER, 1.5, 2.75, .85, 5, true); //2.3 Timeout
    			robot.turret.lockAngle(robot.intake.getCurrentAngle(),98.5);
        		robot.turret.setState(Turret.State.GyroComp);
    		}else{
    			dist.setGoal(robot.dt.getX()+1, ControlWheel.SWERVE, closeHopperY, ControlWheel.FOLLOWER, 1.5, 2.75, .85, 5, false);
    			robot.turret.lockAngle(robot.intake.getCurrentAngle(),-98.5);
        		robot.turret.setState(Turret.State.GyroComp);
        		
    		}
    		robot.retractBallFlap();
    		System.out.println("Turret Goal: " + Double.toString(robot.turret.getGoal()));
    		switch(team){
	    		case BLUE:
	//    			robot.dt.setHeading(180, true);
	    			break;
	    		case RED:
	//    			robot.dt.setHeading(0, true);
	    			break;
    		}
    		delay();
    		//robot.shooter.setGoal(Constants.SHOOTING_SPEED + 100);
    		//robot.shooter.setState(Shooter.Status.STARTED);
    		
    		//Step 2 
    		//Add Timeout 
    		dist.setXPID(Constants.DIST_CONTROLLER_X_P, 0.0, Constants.DIST_CONTROLLER_X_D, Constants.DIST_CONTROLLER_X_FF);
    		dist.setYPID(0.02, 0.0, 0.0, 0.0);
    		//dist.setYPID(Constants.DIST_CONTROLLER_Y_SHORT_P, 0.0, Constants.DIST_CONTROLLER_Y_SHORT_D, Constants.DIST_CONTROLLER_Y_SHORT_FF);
    		if(team == BLUE){
    			dist.setGoal(robot.dt.getX()+((-36)*team), ControlWheel.SWERVE, closeHopperY, ControlWheel.FOLLOWER, 2.0, 1.3, 0.6, 0, true);
    		}else{
    			dist.setGoal(robot.dt.getX()+((-36)*team), ControlWheel.SWERVE, closeHopperY, ControlWheel.FOLLOWER, 2.0, 1.3, 0.6, 0, false);
    		}
    		long time = System.currentTimeMillis();
    		timeout = System.currentTimeMillis() + 850;
    		while(Math.abs(robot.dt.getX()) < 12 && isAutonomous() && System.currentTimeMillis() < timeout){
    			Timer.delay(0.01);
    		}
    		timeout = System.currentTimeMillis() + (1300 - (System.currentTimeMillis() - time));
    		while(isAutonomous() && System.currentTimeMillis() < timeout && robot.intake.getCurrentAngularRate() < 30){
    			Timer.delay(0.01);
    		}
    		dist.disable();
    		dist.blowUpTimeout();
    		//delay();
    		System.out.println("X movement completed in " + Long.toString(System.currentTimeMillis() - time));
    		logger.writeToLog("AUTO POS1 X:" + Double.toString(dist.getXInches()) + " Y:"+ Double.toString(dist.getYInches()));
    		
    		//Step 3
    		//Add swerve + turret lock and then feed vision as a correction
    		/*if(fsm.getTargetVisibility() && Math.abs(fsm.getTargetAngle() - robot.turret.getAngle()) < 3.0){
    			robot.turret.setState(Turret.State.VisionTracking);
    			System.out.println("Vision on");
    			Timer.delay(0.2);
    			robot.turret.lockAngle(Util.BoundPigeonAngle(robot.intake.getCurrentAngle()), robot.turret.getAngle());
    			robot.turret.setState(Turret.State.GyroComp);
    		}*/
    		robot.shooter.setGoal(Constants.SHOOTING_SPEED);
    		robot.shooter.setState(Shooter.Status.STARTED);
    		robot.deployBallFlap();
    		dist.setXPID(0.0005, 0.0, 0.0, 0.1);
    		dist.setYPID(Constants.DIST_CONTROLLER_Y_SHORT_P, 0.0, Constants.DIST_CONTROLLER_Y_SHORT_D, 0.25);
    		dist.blowUpTimeout();
    		if(team == BLUE){
    			dist.setGoal(robot.dt.getX() + (2*team), ControlWheel.SWERVE, Constants.NEAR_HOPPER_Y + 4, ControlWheel.FOLLOWER, 3.0, 1.5, 1.0, 10, true); 
    		}else{
    			dist.setGoal(robot.dt.getX() + (2*team), ControlWheel.SWERVE, Constants.NEAR_HOPPER_Y + 4, ControlWheel.FOLLOWER, 3.0, 1.5, 1.0, 10, false);
    		}
    		timeout = System.currentTimeMillis() + 1250;
    		while(isAutonomous() && System.currentTimeMillis() < timeout){
    			Timer.delay(0.01);
    		}
    		robot.sweeper.turnSweeperOn();
    		robot.intake.intakeForward();
    		
    		delay();
    		robot.shooter.setGoal(robot.shooter.getShooterSpeedForRange(dist.getYInches()));
    		robot.shooter.setState(Shooter.Status.STARTED);
    		System.out.println("Final Shooting Speed: " + Double.toString(robot.shooter.getShooterSpeedForRange(dist.getYInches())))   ;
    		//Step 4
    		logger.writeToLog("Final Distance Y:" + dist.getYInches());
    		
    		
    		/*while (robot.shooter.getStatus()!=Shooter.Status.READY && isAutonomous()){
    			Timer.delay(0.01);
    		}*/
    		timeout = System.currentTimeMillis() + 2500;
    		while(isAutonomous() && System.currentTimeMillis() < timeout){
    			Timer.delay(0.01);
    			
    		}
    		robot.retractBallFlap();
    		timeout = System.currentTimeMillis() + 1000;
    		while(isAutonomous() && System.currentTimeMillis() < timeout){
    			Timer.delay(0.01);
    		}
    		robot.deployBallFlap();
    		timeout = System.currentTimeMillis() + 1500;
    		while(isAutonomous() && System.currentTimeMillis() < timeout){
    			Timer.delay(0.01);
    			
    		}
    		robot.retractBallFlap();
    		timeout = System.currentTimeMillis() + 1000;
    		while(isAutonomous() && System.currentTimeMillis() < timeout){
    			Timer.delay(0.01);
    		}
    		robot.deployBallFlap();
    		timeout = System.currentTimeMillis() + 1500;
    		while(isAutonomous() && System.currentTimeMillis() < timeout){
    			Timer.delay(0.01);
    			
    		}
    		robot.retractBallFlap();
    		timeout = System.currentTimeMillis() + 1000;
    		while(isAutonomous() && System.currentTimeMillis() < timeout){
    			Timer.delay(0.01);
    		}
    		robot.deployBallFlap();
    		while(isAutonomous()){
    			Timer.delay(0.01);
    		}
    		robot.dt.sendInput(0, 0, 0, 0, false, false, false, false);
    		robot.sweeper.stopRoller();
    		robot.sweeper.stopSweeper();
    		robot.intake.intakeStop();
    		robot.shooter.setState(Shooter.Status.OFF);
    		fsm.auto = false;
    		break;
    	case OFF:
    		logger.writeToLog("AUTO *** OFF ***");
    		//dist.setXPID(0.001, 0.0, 0.0, 0.25);
    		dist.setYPID(0.008, 0.0, 0.04, 0.1); //.008 - .04 - .10
    		dist.setXPID(0.005, 0.0, 0.0, 0.0);
    		logger.writeToLog("Y distance before: " + Double.toString(robot.dt.frontRight.getNegatedFollowerWheelInches()) + " X distance: " + Double.toString(robot.dt.getX()));
    		dist.setGoal(robot.dt.getX(), ControlWheel.SWERVE, 79.0, ControlWheel.FOLLOWER, 0, 5.0, .85, 10, true);
    		delay();
    		logger.writeToLog("Y distance: " + Double.toString(dist.getYInches()));
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