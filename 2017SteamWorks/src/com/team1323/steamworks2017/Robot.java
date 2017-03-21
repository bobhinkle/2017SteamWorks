package com.team1323.steamworks2017;

import ControlSystem.FSM;
import ControlSystem.RoboSystem;
import Helpers.InterpolatingDouble;
import Helpers.Looper;
import IO.Logger;
import IO.TeleController;
import SubSystems.DistanceController;
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
	final String two_gear    = "two_gear";
	final String near_hopper = "turn_shoot";
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
    	/** Start at center position. Move field-forward to score gear. Move field-backward and rotate to 90 or 270 degrees
    	 *   to face another gear on the field. Intake gear from floor. Rotate to 180 degrees and move field-forward to score
    	 *   gear. Shoot balls. */
		TWO_GEAR,
    	/** Deploy hoppers on boiler side of field. Move under the outside hopper to receive balls. Shoot balls. */
		NEAR_HOPPER,
    	/** Deploy hoppers on not-boiler side of field. Move under the outside hopper to receive balls. Move to field x-center,
    	 *    */
		FAR_HOPPER,
		NEAR_HOPPER_INSIDE,
		NEAR_HOPPER_OUTSIDE,
		NEAR_HOPPER_RECEIVE,
		NEAR_HOPPER_CLOSE_INSIDE
    }
	
    public Robot() {
    	autoSelect = new SendableChooser();
        autoSelect.addDefault("Off", off);
        //autoSelect.addObject("One_Gear", one_gear);
        //autoSelect.addObject("Two_Gear", two_gear);
        autoSelect.addObject("Turn_Shoot", near_hopper);
        //autoSelect.addObject("Near_Hopper_Inside", near_hopper_inside);
        //autoSelect.addObject("Near_Hopper_Outside", near_hopper_outside);
        //autoSelect.addObject("Far_Hopper", far_hopper);
        autoSelect.addObject("Near_Hopper_Receive", near_hopper_receive);
        autoSelect.addObject("Near_Hopper_Close_Inside", near_hopper_close_inside);
        SmartDashboard.putData(" Select Auto ", autoSelect);
        allianceSelect = new SendableChooser();
        allianceSelect.addDefault("Blue", blue);
        allianceSelect.addObject("Red", red);
        SmartDashboard.putData(" Select  Alliance ", allianceSelect);
        
        robot = RoboSystem.getInstance();
        controllers = TeleController.getInstance();
        fsm = FSM.getInstance();
        dist = DistanceController.getInstance();
        robot.intake.setPresetAngles(0);
        Timer.delay(0.1);
        robot.dt.resetCoord(Swerve.AnglePresets.ZERO);
        Timer.delay(0.1);
    //    robot.intake.setPresetAngles(90);
    	/** Autonomous subroutine that the user has selected to run */
    	String autoSelected = (String) autoSelect.getSelected();
    	SmartDashboard.putString(" Auto Selected ", autoSelected);
/*    		switch(autoSelected){
    			case one_gear:
    	            robot.intake.setPresetAngles(180);
    	        	robot.dt.resetCoord(Swerve.AnglePresets.ONE_EIGHTY);
    				break;
    			case two_gear:
    	            robot.intake.setPresetAngles(180);
    	        	robot.dt.resetCoord(Swerve.AnglePresets.ONE_EIGHTY);
    				break;
    			case near_hopper:
    	            robot.intake.setPresetAngles(270);
    	        	robot.dt.resetCoord(Swerve.AnglePresets.TWO_SEVENTY);
    				break;
    			case far_hopper:
    	            robot.intake.setPresetAngles(270);
    	        	robot.dt.resetCoord(Swerve.AnglePresets.TWO_SEVENTY);
    				break;
    			case off:
    				robot.intake.setPresetAngles(0);
    				robot.dt.resetCoord(Swerve.AnglePresets.ZERO);
    				break;
    	*///	}
    	
    	logger = Logger.getInstance();
    	robot.turret.resetAngle(90);
    }

	@Override
	public void robotInit() {
		mVisionServer.addVisionUpdateReceiver(VisionProcessor.getInstance());
		mEnabledLooper.register(VisionProcessor.getInstance());
		VisionServer.getInstance();
		mEnabledLooper.start();
	}
    /** The autonomous routine, which calls the selected autonomous subroutine */
    public void autonomous() {
    	String autoSelected = (String) autoSelect.getSelected();
    	String allianceSelected = (String) allianceSelect.getSelected();
//    	robot.intake._pidgey.SetFusedHeading(180.0);
//    	robot.dt.resetCoord(AnglePresets.ONE_EIGHTY);
//    	robot.dt.setHeading(0.0);
    	int alliance = BLUE;
    	switch(allianceSelected){
	    	case blue:
	    		alliance = BLUE;
	    		break;
	    	case red:
	    		alliance = RED;
	    		break;
	    	default:
	    		alliance = BLUE;
	    		break;
    	}
    		switch(autoSelected){
    			case one_gear:
    				robot.intake.setPresetAngles(0);
    		        Timer.delay(0.1);
    		        robot.dt.resetCoord(Swerve.AnglePresets.ZERO);
    	            Timer.delay(0.1);
    	            robot.intake.setPresetAngles(180);
    	            Timer.delay(0.1);
    	            robot.turret.setState(Turret.State.VisionTracking);
    	            executeAuto(AUTO.ONE_GEAR, alliance);
    				
    				break;
    			case two_gear:
    	        //    robot.intake.setPresetAngles(180);
    	      //  	robot.dt.resetCoord(Swerve.AnglePresets.ONE_EIGHTY);			
    				executeAuto(AUTO.TWO_GEAR, alliance);
    				break;
    			case near_hopper:
    	            switch(allianceSelected){
	            	case blue:
	            		initHeading(90);
	    	            Timer.delay(0.1);
	            		executeAuto(AUTO.NEAR_HOPPER, BLUE);
	            		break;
	            	case red:
	            		initHeading(270);
	    	            Timer.delay(0.1);
	            		executeAuto(AUTO.NEAR_HOPPER, RED);
	            		break;
	            	default:
	            		initHeading(90);
	    	            Timer.delay(0.1);
	            		executeAuto(AUTO.NEAR_HOPPER, BLUE);
	            		break;
	            }
    				break;
    			case near_hopper_inside:
    	            switch(allianceSelected){
	            	case blue:
	            		initHeading(90);
	            		executeAuto(AUTO.NEAR_HOPPER_INSIDE, BLUE);
	            		break;
	            	case red:
	            		initHeading(270);
	            		executeAuto(AUTO.NEAR_HOPPER_INSIDE, RED);
	            		break;
	            	default:
	            		initHeading(90);
	            		executeAuto(AUTO.NEAR_HOPPER_INSIDE, BLUE);
	            		break;
	            }
    			case near_hopper_close_inside:
    	            switch(allianceSelected){
	            	case blue:
	            		initHeading(90);
	            		executeAuto(AUTO.NEAR_HOPPER_CLOSE_INSIDE, BLUE);
	            		break;
	            	case red:
	            		initHeading(270);
	            		executeAuto(AUTO.NEAR_HOPPER_CLOSE_INSIDE, RED);
	            		break;
	            	default:
	            		initHeading(90);
	            		executeAuto(AUTO.NEAR_HOPPER_CLOSE_INSIDE, BLUE);
	            		
	            		break;
	            }
    				break;
    			case near_hopper_outside:
    	            switch(allianceSelected){
	            	case blue:
	            		initHeading(90);
	            		executeAuto(AUTO.NEAR_HOPPER_OUTSIDE, BLUE);
	            		break;
	            	case red:
	            		initHeading(270);
	            		executeAuto(AUTO.NEAR_HOPPER_OUTSIDE, RED);
	            		break;
	            	default:
	            		initHeading(90);
	            		executeAuto(AUTO.NEAR_HOPPER_OUTSIDE, BLUE);
	            		break;
	            }
    				break;
    			case near_hopper_receive:
    	            switch(allianceSelected){
	            	case blue:
	            		initHeading(90);
	            		executeAuto(AUTO.NEAR_HOPPER_RECEIVE, BLUE);
	            		break;
	            	case red:
	            		initHeading(270);
	            		executeAuto(AUTO.NEAR_HOPPER_RECEIVE, RED);
	            		break;
	            	default:
	            		initHeading(90);
	            		executeAuto(AUTO.NEAR_HOPPER_RECEIVE, BLUE);
	            		break;
	            }
    				break;
    			case far_hopper:
    	//            robot.intake.setPresetAngles(270);
    //	        	robot.dt.resetCoord(Swerve.AnglePresets.TWO_SEVENTY);
    				executeAuto(AUTO.FAR_HOPPER, 1);
    				break;
    			case off:
    				break;
    		}

    }
//    private void setStartHeading(double angle) {
 //   	robot.intake._pidgey.SetFusedHeading(angle);
//    	robot.dt.setHeading(angle);
//    }

/*    public void rotate(double angle){
    	long timeout = System.currentTimeMillis() + 1000;
    	robot.dt.setHeading(angle,true);
		while(!robot.dt.headingOnTarget() && isAutonomous() && (timeout > System.currentTimeMillis())){
			robot.dt.sendInput(0, 0, 0, 0, false, true, false);
			Timer.delay(0.01);
		}
    }*/
    
    private void delay() {
		while(!dist.onTarget() && isAutonomous()){
			Timer.delay(0.01);
		}
    }
    public void executeAuto(AUTO autoSelect, int team){
    	robot.intake.deployWings();
    	robot.gearIntake.gearExtender();
    	// 1 is blue, -1 is red
    	switch(autoSelect){
    	case ONE_GEAR:
    		robot.dt.setHeading(180, true);
    		robot.turret.setState(Turret.State.VisionTracking);
    		robot.dt.sendInput(0.0, -0.35, 0.0, 0.0, false, false, false, false);
    		while(!fsm.isAtDistance(75) && isAutonomous()){
    			Timer.delay(0.01);
    			logger.writeToLog("Vision distance Auto: " + Double.toString(fsm.getTargetDistance()));
    		}
    		robot.dt.sendInput(0, 0, 0, 0, false, false, false, false);
    		
    		
    		
    		
    		
    		/*robot.turret.setState(Turret.State.CalculatedTracking);
    		dist.setGoal(Constants.TWO_G_PEG_X+1, Constants.TWO_G_PEG_Y -4, 1, 3, 0.5);
//        	robot.turret.setAngle(65);
    		delay();
    		Timer.delay(0.25);
    		robot.dt.sendInput(0.0, 0.4, 0.0, 0.0, false, false, false);
    		Timer.delay(1.25);
    		robot.dt.sendInput(0.0, 0.0, 0.0, 0.0, false, false, false);
    		dist.setGoal(0.0, Constants.TWO_G_PEG_Y - 10, 2.0, 1, 0.5);
    		delay();
    		dist.setGoal(0.0, Constants.TWO_G_PEG_Y - 15, 1.0, 2, 0.5);
    		robot.dt.setHeading(270, true);
    		delay();
    		robot.dt.sendInput(0, 0, 0, 0, false, false, false);
    		robot.turret.setState(Turret.State.VisionTracking);
    		Timer.delay(0.75);
    		robot.turret.lockAngle(robot.intake.getCurrentAngle());
    		robot.turret.setState(Turret.State.GyroComp);
    		robot.shooter.setGoal(robot.shooter.getShooterSpeedForRange(fsm.getTargetDistance()));
    		robot.shooter.setState(Shooter.Status.STARTED);
    		while (robot.shooter.getStatus()!=Shooter.Status.READY && isAutonomous()){
    			Timer.delay(.01);
    		}
    		robot.sweeper.forwardRoller();
    		robot.sweeper.reducedForward();
    		Timer.delay(3);
    		robot.sweeper.stopRoller();
    		robot.sweeper.stopSweeper();
    		Timer.delay(1);
    		robot.shooter.stop();*/
    		break;
    	case TWO_GEAR:
    		//robot.intake._pidgey.SetFusedHeading(0.0);
        	//robot.dt.setHeading(0.0,false);    
        	//rotate(180);
    		// intake does something to get the gear?
    		/*robot.turret.setState(Turret.State.CalculatedTracking);
        	dist.setGoal(Constants.TWO_G_PEG_X, Constants.TWO_G_PEG_Y, 1, 2, 0.9);
        	robot.turret.setAngle(65);
    		delay();
    		robot.turret.setState(Turret.State.VisionTracking);
    		// reverse gear intake to score
    		// stop gear intake
    		robot.dt.setHeading(90, true);
    		dist.setGoal(Constants.TWO_G_PEG_X, Constants.TWO_G_PICKUP_Y, 1, 2, 0.9);
    		delay();
   		// move gear intake down
    		// turn gear intake on
    		dist.setGoal(Constants.TWO_G_PICKUP_X, Constants.TWO_G_PICKUP_Y, 1, 1.0, 0.9);
    		delay();
    		// gear is taken in here
    		// turn gear intake off?
    		// move gear intake up
    		robot.dt.setHeading(180, true);
    		dist.setGoal(Constants.TWO_G_RETURN_X, Constants.TWO_G_RETURN_Y, 2, 3, 0.9);
    		robot.turret.setState(Turret.State.GyroComp);
    		robot.shooter.setState(Shooter.Status.STARTED);
    		while (robot.shooter.getStatus()!=Shooter.Status.READY && isAutonomous()){
    			Timer.delay(.01);
    		}
    		// reverse gear intake to score
    		// stop gear intake
    		robot.sweeper.forwardRoller();
    		robot.sweeper.forwardSweeper();
    		Timer.delay(5);
    		robot.sweeper.stopRoller();
    		robot.sweeper.stopSweeper();
    		Timer.delay(1);
    		robot.shooter.setState(Shooter.Status.OFF);*/
    		break;
    	case NEAR_HOPPER:
    		if(team == BLUE){
    			robot.dt.setHeading(270, true);
    		}else{
    			robot.dt.setHeading(90, true);
    		}
    		
    		robot.turret.setState(Turret.State.VisionTracking);
    		robot.intake.intakeForward();
    		Timer.delay(1.5);
    		robot.shooter.setGoal(2850.0);
    		logger.writeToLog("Vision distance Auto: " + Double.toString(fsm.getTargetDistance()));
    		robot.turret.lockAngle(robot.intake.getCurrentAngle());
    		robot.turret.setState(Turret.State.GyroComp);
    		robot.shooter.setState(Shooter.Status.STARTED);
    		while (robot.shooter.getStatus()!=Shooter.Status.READY && isAutonomous()){
    			Timer.delay(0.01);
    		}
    		robot.sweeper.turnSweeperOn();
    		//robot.intake.reducedRightForward();
    		Timer.delay(5);
    		dist.setGoal(robot.dt.getX(), Constants.NEAR_HOPPER_DEPLOY_Y + 16, 4.0, 3.0, 0.9, 10);
    		robot.sweeper.stopRoller();
    		robot.sweeper.stopSweeper();
    		robot.intake.intakeStop();
    		robot.shooter.setState(Shooter.Status.OFF);
    		delay();
    		//Move in a positive y direction towards the gear peg
    		/*robot.turret.setState(Turret.State.VisionTracking);
    		robot.intake.intakeReverse();
    		robot.dt.setHeading(240, true);
    		robot.turret.setAngle(-80);
    		dist.setGoal(Constants.NEAR_HOPPER_FIRST_X, Constants.NEAR_HOPPER_FIRST_Y, 2.0, 3.0, 0.8);    		
    		delay();
    		//Move backward just a bit to place the gear on the peg
    		dist.setGoal(Constants.NEAR_HOPPER_PEG_X, Constants.NEAR_HOPPER_PEG_Y, 2.0, 3.0, 0.8);
    		robot.intake.intakeStop();
    		delay();
    		robot.dt.sendInput(0.35, 0.35, 0.0, 0.0, false, false, false);
    		Timer.delay(1.0);
    		robot.dt.sendInput(0.0, 0.0, 0.0, 0.0, false, false, false);
    		robot.dt.setHeading(270, true);
    		robot.turret.setState(Turret.State.VisionTracking);
    		dist.setGoal(Constants.NEAR_HOPPER_DEPLOY_X, Constants.NEAR_HOPPER_DEPLOY_Y, 3.0, 4.0, 0.85);
    		delay();
    		Timer.delay(1);    		
    		dist.setGoal(robot.dt.getX()+4, robot.dt.getY()+10, 2.0, 2.0, 0.9);
    		delay();
    		robot.shooter.setGoal(robot.shooter.getShooterSpeedForRange(fsm.getTargetDistance()));
    		robot.turret.setState(Turret.State.Off);
    		robot.shooter.setState(Shooter.Status.STARTED);
    		while (robot.shooter.getStatus()!=Shooter.Status.READY && isAutonomous()){
    			Timer.delay(.01);
    		}
    		robot.sweeper.forwardRoller();
    		robot.sweeper.forwardSweeper();
    		Timer.delay(5);
    		robot.sweeper.stopRoller();
    		robot.sweeper.stopSweeper();
    		Timer.delay(1);
    		robot.shooter.setState(Shooter.Status.OFF);
    		//Turn on the intake so as to collect as many balls as possible throughout autonomous
    		//robot.intake.intakeForward();
    		//Move forward and reach the hopper, deploying it in the process
    		//dist.setGoal(-10,50, 2.0,5.0, 0.7);
    		//delay();
    		//Move in a positive x direction to fall in the path of the hopper's balls
    		//dist.setGoal(5,robot.dt.frontLeft.getY(), 2.0,5.0, 0.7);
    		
    		//robot.intake.intakeStop();*/
    		break;
    	case NEAR_HOPPER_INSIDE:
    		//robot.turret.setAngle(-75);
    		//robot.turret.setState(Turret.State.VisionTracking);
    		//robot.intake.intakeReverse();
    		dist.setGoal(robot.dt.getX(), Constants.NEAR_HOPPER_DEPLOY_Y, 4.0, 2.75, 0.7, 20);
    		delay();
    		
    		/*dist.setGoal(robot.dt.getX()+(-24 + 4)*team, robot.dt.getY(), 3.0, 0.9, 0.9, 10);
    		delay();
    		dist.setGoal(robot.dt.getX() + 6*team, robot.dt.getY() + 18, 1.0, 1.5, 0.95, 10);
    		delay();
    		robot.shooter.setGoal(robot.shooter.getShooterSpeedForRange(fsm.getTargetDistance()));
    		robot.turret.setState(Turret.State.Off);
    		robot.shooter.setState(Shooter.Status.STARTED);
    		while (robot.shooter.getStatus()!=Shooter.Status.READY && isAutonomous()){
    			Timer.delay(0.01);
    		}
    		robot.sweeper.forwardRoller();
    		robot.sweeper.forwardSweeper();
    		robot.intake.reducedForward();
    		Timer.delay(6);
    		robot.sweeper.stopRoller();
    		robot.sweeper.stopSweeper();
    		robot.intake.intakeStop();
    		Timer.delay(1);
    		robot.shooter.setState(Shooter.Status.OFF);*/
    		break;
    	case NEAR_HOPPER_CLOSE_INSIDE:
    		
    		dist.setGoal(robot.dt.getX(), /*/Constants.NEAR_HOPPER_DEPLOY_Y-14/*/84, 2.2, 2.1, .9, 2);
    		
    		if(team == BLUE){
    			robot.dt.setHeading(90, true);
    			robot.turret.setAngle(105);
    		}else{
    			robot.dt.setHeading(270, true);
    			robot.turret.setAngle(-85);
    		}
    		delay();
    		robot.turret.setState(Turret.State.VisionTracking);
    		dist.setGoal(robot.dt.getX()+((-65)*team), 84, 2.0, 1.35, 1.0, 5);
    		delay();
    		logger.writeToLog("AUTO POS1 X:" + Double.toString(robot.dt.getX()) + " Y:"+ Double.toString(robot.dt.getY()));
    		
    		InterpolatingDouble power = Constants.kDistanceMap.getInterpolated(new InterpolatingDouble(fsm.getTargetDistance()));
    		robot.dt.sendInput(0.0, -power.value, 0.0, 0.0, false, false, false, false);
    		if(team == BLUE){
    			robot.dt.setHeading(90, true);
    		}else{
    			robot.dt.setHeading(270, true);
    		}
    		long timeout = System.currentTimeMillis();
    		double currentDist = robot.dt.getRobotYInch();
    		while((!fsm.isAtDistance(81) || (currentDist - robot.dt.getRobotYInch()<2)) && isAutonomous() && (System.currentTimeMillis() - timeout) < 1000){
    			if(fsm.getTargetVisibility()){
    				power = Constants.kDistanceMap.getInterpolated(new InterpolatingDouble(fsm.getTargetDistance()));
    				robot.dt.sendInput(0.015, -power.value, 0.0, 0.0, false, false, false, false);
    			}
    			else{
    				robot.dt.sendInput(0.0, -0.3, 0.0, 0.0, false, false, false, false);
    			}
        		
        		logger.writeToLog("Vision " + Boolean.toString(fsm.getTargetVisibility()));
        		logger.writeToLog("Vision Distance: " + Double.toString(fsm.getTargetDistance()));
    			Timer.delay(0.01);    			
    		}
    		if(!fsm.getTargetVisibility()){
    			if(team == BLUE){
    				robot.turret.setAngle(105+(90-Util.boundAngle0to360Degrees(robot.intake.getCurrentAngle())));
    			}
    		}
    		logger.writeToLog("Vision distance Auto: " + Double.toString(fsm.getTargetDistance()) + " Power: " + Double.toString(power.value));
    		robot.dt.sendInput(0, 0.3, 0, 0, false, false, false, false);
    		Timer.delay(0.25);
    		robot.dt.sendInput(0, 0.0, 0, 0, false, false, false, false);
    		logger.writeToLog("AUTO POS1 X:" + Double.toString(robot.dt.getX()) + " Y:"+ Double.toString(robot.dt.getY()));
    		/* 
    		while(!robot.turret.onTarget() && isAutonomous()){
    			Timer.delay(0.01);
    		}    		
    		Timer.delay(0.5);
    		robot.shooter.setGoal(robot.shooter.getShooterSpeedForRange(fsm.getTargetDistance()));
    		logger.writeToLog("Vision distance Auto: " + Double.toString(fsm.getTargetDistance()));
    		robot.turret.lockAngle(robot.intake.getCurrentAngle());
    		robot.turret.setState(Turret.State.GyroComp);
    		robot.shooter.setState(Shooter.Status.STARTED);
    		while (robot.shooter.getStatus()!=Shooter.Status.READY && isAutonomous()){
    			Timer.delay(0.01);
    		}
    		robot.sweeper.turnSweeperOn();
    		while(isAutonomous()){
    			Timer.delay(0.01);
    			
    		}
    		robot.sweeper.stopRoller();
    		robot.sweeper.stopSweeper();
    		robot.intake.intakeStop();
    		robot.shooter.setState(Shooter.Status.OFF);*/
    		break;
    	case NEAR_HOPPER_OUTSIDE:
    		robot.turret.setAngle(-75);
    		robot.turret.setState(Turret.State.VisionTracking);
    		//robot.intake.intakeReverse();
    		dist.setGoal(robot.dt.getX(), Constants.NEAR_HOPPER_DEPLOY_Y, 4.0, 3.0, 0.7, 10);
    		delay();
    		dist.setGoal(robot.dt.getX()+((-24-25)*team), robot.dt.getY(), 3.0, /**/1.25/*/0.9/**/, 0.8, 10);
    		delay();
    		dist.setGoal(robot.dt.getX() + (3*team), robot.dt.getY() + 18, 1.0, 1.5, 0.95, 10);
    		delay();

        	/** Commented the following to test autos while shooter is out of commission */
    		
    		Timer.delay(1.5);
    		robot.shooter.setGoal(robot.shooter.getShooterSpeedForRange(fsm.getTargetDistance()));
    		robot.turret.setState(Turret.State.Off);
    		robot.shooter.setState(Shooter.Status.STARTED);
    		while (robot.shooter.getStatus()!=Shooter.Status.READY && isAutonomous()){
    			Timer.delay(0.01);
    		}
    		robot.sweeper.forwardRoller();
    		robot.sweeper.forwardSweeper();
    		robot.intake.reducedForward();
    		Timer.delay(6);
    		robot.sweeper.stopRoller();
    		robot.sweeper.stopSweeper();
    		robot.intake.intakeStop();
    		Timer.delay(1);
    		robot.shooter.setState(Shooter.Status.OFF);
    		break;
    	case NEAR_HOPPER_RECEIVE:
    		long startTime = System.currentTimeMillis();
    		dist.setGoal(robot.dt.getX(), Constants.NEAR_HOPPER_DEPLOY_Y + 25, 4.0, 2.6, 0.91, 5);
    		delay();
    		dist.setGoal(robot.dt.getX()+(-72*team), robot.dt.getY(), 2.0, 3.0, 0.90, 5);
    		delay();
    		robot.intake.intakeForward();
    		while((System.currentTimeMillis() - startTime) < timeToWaitMs){
    			logger.writeToLog("Time Passed: " + Long.toString(System.currentTimeMillis() - startTime));
    			Timer.delay(0.01);
    		}
    		dist.setGoal(robot.dt.getX()+(62*team), robot.dt.getY() - 10, 2.0, 2.0, 0.90, 2);
    		delay();
    		dist.setGoal(robot.dt.getX()+(-5*team), robot.dt.getY() - 75, 3.0, 2.0, 0.9, 0);
    		if(team == BLUE){
        		robot.dt.setHeading(robot.intake.getCurrentAngle(), true);
        		}else{
        			robot.dt.setHeading(robot.intake.getCurrentAngle(), true);
        		}
    		
    		delay();
    		if(team == BLUE){
    			robot.turret.setAngle((270 - Util.boundAngle0to360Degrees(robot.intake.getCurrentAngle())) + (-12));
    		}else{
    			robot.turret.setAngle((90 - Util.boundAngle0to360Degrees(robot.intake.getCurrentAngle())) + (12));
    		}
    		//robot.turret.setAngle(30 * team);
    		robot.turret.setState(Turret.State.VisionTracking);
        	/** Commented the following to test autos while shooter is out of commission */
    		while(!robot.turret.onTarget() && isAutonomous()){
    			Timer.delay(0.01);
    		}    		
    		Timer.delay(1);
    		logger.writeToLog("Vision distance Auto: " + Double.toString(fsm.getTargetDistance()));
    		robot.shooter.setGoal(robot.shooter.getShooterSpeedForRange(fsm.getTargetDistance()));
    		robot.turret.lockAngle(robot.intake.getCurrentAngle());
    		robot.turret.setState(Turret.State.GyroComp);
    		robot.shooter.setState(Shooter.Status.STARTED);
    		while (robot.shooter.getStatus()!=Shooter.Status.READY && isAutonomous()){
    			Timer.delay(0.01);
    		}
    		robot.sweeper.turnSweeperOn();
    		//robot.intake.reducedRightForward();
    		while(isAutonomous()){
    			Timer.delay(0.01);
    			
    		}
    		robot.sweeper.stopRoller();
    		robot.sweeper.stopSweeper();
    		robot.intake.intakeStop();
    		robot.shooter.setState(Shooter.Status.OFF);
    		break;
    	case FAR_HOPPER:
    	// Deploy the intakes
    		/*robot.turret.setState(Turret.State.CalculatedTracking);
    		robot.intake.intakeReverse();
    		robot.dt.setHeading(240, true);
    		robot.turret.setAngle(-80);
    		dist.setGoal(Constants.NEAR_HOPPER_FIRST_X, Constants.NEAR_HOPPER_FIRST_Y, 4.0, 3.0, 0.8);    		
    		delay();
    		//Move backward just a bit to place the gear on the peg
    		dist.setGoal(Constants.NEAR_HOPPER_PEG_X-5, Constants.NEAR_HOPPER_PEG_Y, 3.0, 1.8, 0.8);
    		robot.intake.intakeStop();
    		delay();
    		robot.dt.setHeading(270, true);
    		robot.turret.setState(Turret.State.VisionTracking);
    		dist.setGoal(Constants.NEAR_HOPPER_DEPLOY_X-17, Constants.NEAR_HOPPER_DEPLOY_Y, 3.0, 1.8, 0.9);
    		delay();
    		dist.setGoal(Constants.NEAR_HOPPER_DEPLOY_X+2, Constants.NEAR_HOPPER_PICKUP_Y, 0.5, 1, 0.9); // Y was Near Hopper Deploy Y
    		delay();
//    		robot.shooter.setState(Shooter.Status.STARTED);
    		dist.setGoal(Constants.NEAR_HOPPER_DEPLOY_X-17, Constants.NEAR_HOPPER_PICKUP_Y, 0.7, 1.5, 0.9);
    		delay();
    		Timer.delay(1.5);
    		robot.dt.setHeading(180, true);
    		dist.setGoal(50, 10, 3.0, 1.5, 0.9);	// extra waypoint to keep from running into the hexagon thing
    		delay();
    		dist.setGoal(52.5, 20, 3.0, 4.0, 0.9);
    		delay();
    		robot.turret.setState(Turret.State.GyroComp);
    		robot.shooter.setState(Shooter.Status.STARTED);
    		while (robot.shooter.getStatus()!=Shooter.Status.READY && isAutonomous()){
    			Timer.delay(.01);
    		}
    		robot.sweeper.forwardRoller();
    		robot.sweeper.forwardSweeper();
    		Timer.delay(5);
    		robot.sweeper.stopRoller();
    		robot.sweeper.stopSweeper();
    		Timer.delay(1);
    		robot.shooter.stop();*/
    		break;
    	case OFF: break;
    	default: break;
    }
    	while(isAutonomous())
    	{Timer.delay(0.1);}
    }/**/
    
    private void initHeading(double angle){
    	robot.intake.setPresetAngles(angle);
        Timer.delay(0.1);
        robot.dt.resetCoord(Swerve.AnglePresets.ZERO);
        Timer.delay(0.1);
    }
    
    public void operatorControl() {
    	dist.disable();
    	robot.shooter.stop();
        while (isOperatorControl() && isEnabled()) {        	
        	controllers.update();
            Timer.delay(0.01);		//10ms Loop Rate
        }
    }
}