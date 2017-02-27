package com.team1323.steamworks2017;

import ControlSystem.FSM;
import ControlSystem.RoboSystem;
import Helpers.Looper;
import IO.TeleController;
import SubSystems.DistanceController;
import SubSystems.Intake;
import SubSystems.Shooter;
import SubSystems.Swerve;
import SubSystems.VisionProcessor;
import SubSystems.VisionServer;
import Utilities.Constants;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends SampleRobot {
	private RoboSystem robot = RoboSystem.getInstance();
	private TeleController controllers;
	private FSM fsm = FSM.getInstance();
	final String off = "off";
	final String one_gear    = "one_gear";
	final String two_gear    = "two_gear";
	final String near_hopper = "near_hopper";
	final String far_hopper  = "far_hopper";
	SendableChooser autoSelect;
	private DistanceController dist;
	VisionServer mVisionServer = VisionServer.getInstance();
	// Enabled looper is called at 100Hz whenever the robot is enabled
    Looper mEnabledLooper = new Looper();
    // Disabled looper is called at 100Hz whenever the robot is disabled
    Looper mDisabledLooper = new Looper();
	public static enum AUTO{
    	OFF,ONE_GEAR,TWO_GEAR,NEAR_HOPPER,FAR_HOPPER
    }
    @SuppressWarnings("unchecked")
	public Robot() {
    	autoSelect = new SendableChooser();
        autoSelect.addDefault("Off", off);
        autoSelect.addObject("One_Gear", one_gear);
        autoSelect.addObject("Two_Gear", two_gear);
        autoSelect.addObject("Near_Hopper", near_hopper);
        autoSelect.addObject("Far_Hopper", far_hopper);
        SmartDashboard.putData("Select Auto", autoSelect);  
        robot = RoboSystem.getInstance();
        controllers = TeleController.getInstance();
        fsm = FSM.getInstance(); 
        dist = DistanceController.getInstance();
        robot.intake.setPresetAngles(Intake.AnglePresets.ZERO);
    	robot.dt.resetCoord(Swerve.AnglePresets.ZERO);
        
    	String autoSelected = (String) autoSelect.getSelected();
    	SmartDashboard.putString("AutoSelected", autoSelected);
    		switch(autoSelected){
    			case one_gear:
    	            robot.intake.setPresetAngles(Intake.AnglePresets.ONE_EIGHTY);
    	        	robot.dt.resetCoord(Swerve.AnglePresets.ONE_EIGHTY);
    				break;
    			case two_gear:
    	            robot.intake.setPresetAngles(Intake.AnglePresets.ONE_EIGHTY);
    	        	robot.dt.resetCoord(Swerve.AnglePresets.ONE_EIGHTY);
    				break;
    			case near_hopper:
    	            robot.intake.setPresetAngles(Intake.AnglePresets.TWO_SEVENTY);
    	        	robot.dt.resetCoord(Swerve.AnglePresets.TWO_SEVENTY);
    				break;
    			case far_hopper:
    	            robot.intake.setPresetAngles(Intake.AnglePresets.TWO_SEVENTY);
    	        	robot.dt.resetCoord(Swerve.AnglePresets.TWO_SEVENTY);
    				break;
    			case off:
    				robot.intake.setPresetAngles(Intake.AnglePresets.ZERO);
    				robot.dt.resetCoord(Swerve.AnglePresets.ZERO);
    				break;
    		}
//        robot.intake._pidgey.
/*        SmartDashboard.putBoolean("Manual Wheel Headings?", true);
		SmartDashboard.putNumber("Manual Heading 1", 0); 
        SmartDashboard.putNumber("Manual Heading 2", 0);
        SmartDashboard.putNumber("Manual Heading 3", 0);
        SmartDashboard.putNumber("Manual Heading 4", 0);
/**/        
    }

	@Override
	public void robotInit() {
		mVisionServer.addVisionUpdateReceiver(VisionProcessor.getInstance());
		mEnabledLooper.register(VisionProcessor.getInstance());
		VisionServer.getInstance();
		mEnabledLooper.start();
	}
    
    public void autonomous() {
    	String autoSelected = (String) autoSelect.getSelected();
//    	robot.intake._pidgey.SetFusedHeading(180.0);
//    	robot.dt.resetCoord();
//    	robot.dt.setHeading(0.0);
    		switch(autoSelected){
    			case one_gear:
    	            //robot.intake.setPresetAngles(Intake.AnglePresets.ONE_EIGHTY);
    	        	//robot.dt.resetCoord(Swerve.AnglePresets.ONE_EIGHTY);
    				executeAuto(AUTO.ONE_GEAR);
    				break;
    			case two_gear:
    	        //    robot.intake.setPresetAngles(Intake.AnglePresets.ONE_EIGHTY);
    	      //  	robot.dt.resetCoord(Swerve.AnglePresets.ONE_EIGHTY);
    				executeAuto(AUTO.TWO_GEAR);
    				break;
    			case near_hopper:
    	    //        robot.intake.setPresetAngles(Intake.AnglePresets.TWO_SEVENTY);
    	  //      	robot.dt.resetCoord(Swerve.AnglePresets.TWO_SEVENTY);
    				executeAuto(AUTO.NEAR_HOPPER);
    				break;
    			case far_hopper:
    	//            robot.intake.setPresetAngles(Intake.AnglePresets.TWO_SEVENTY);
    //	        	robot.dt.resetCoord(Swerve.AnglePresets.TWO_SEVENTY);
    				executeAuto(AUTO.FAR_HOPPER);
    				break;
    			case off:
//    				robot.intake.setPresetAngles(Intake.AnglePresets.ZERO);
  //  				robot.dt.resetCoord(Swerve.AnglePresets.ZERO);
    				executeAuto(AUTO.OFF);
    				break;
    		}
    }
    private void setStartHeading(double angle) {
 //   	robot.intake._pidgey.SetFusedHeading(angle);
//    	robot.dt.setHeading(angle);
    }

    public void rotate(double angle){
    	long timeout = System.currentTimeMillis() + 1000;
    	robot.dt.setHeading(angle,true);
		while(!robot.dt.headingOnTarget() && isAutonomous() && (timeout > System.currentTimeMillis())){
			robot.dt.sendInput(0, 0, 0, 0, false, true, false);
			Timer.delay(0.01);
		}
    }
    
    private void delay() {
		while(!dist.onTarget() && isAutonomous()){
			Timer.delay(0.01);
		}
    }
    public void executeAuto(AUTO autoSelect){
    	    	
    	switch(autoSelect){
    	case ONE_GEAR:
    		dist.setGoal(Constants.TWO_G_PEG_X, Constants.TWO_G_PEG_Y, 1, 5, 0.5);
        	robot.turret.setAngle(65);
    		delay();
    		robot.shooter.setState(Shooter.Status.STARTED);
    		robot.sweeper.forwardRoller();
    		robot.sweeper.forwardSweeper();
    		Timer.delay(5);
    		robot.sweeper.stopRoller();
    		robot.sweeper.stopSweeper();
    		Timer.delay(1);
    		robot.shooter.setState(Shooter.Status.OFF);
    		break;
    	case TWO_GEAR:
    		//robot.intake._pidgey.SetFusedHeading(0.0);
        	//robot.dt.setHeading(0.0,false);    
        	//rotate(180);
    		// intake does something to get the gear?
        	dist.setGoal(Constants.TWO_G_PEG_X, Constants.TWO_G_PEG_Y, 1, 2, 0.9);
        	robot.turret.setAngle(65);
    		delay();
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
    		robot.shooter.setState(Shooter.Status.STARTED);
    		delay();
    		// reverse gear intake to score
    		// stop gear intake
    		robot.sweeper.forwardRoller();
    		robot.sweeper.forwardSweeper();
    		Timer.delay(5);
    		robot.sweeper.stopRoller();
    		robot.sweeper.stopSweeper();
    		Timer.delay(1);
    		robot.shooter.setState(Shooter.Status.OFF);
    		
    		break;
    	case NEAR_HOPPER:
    		//Move in a positive y direction towards the gear peg
    		robot.intake.intakeReverse();
    		robot.dt.setHeading(240, true);
    		robot.turret.setAngle(-80);
    		dist.setGoal(Constants.NEAR_HOPPER_FIRST_X, Constants.NEAR_HOPPER_FIRST_Y, 4.0, 3.0, 0.8);    		
    		delay();
    		//Move backward just a bit to place the gear on the peg
    		dist.setGoal(Constants.NEAR_HOPPER_PEG_X, Constants.NEAR_HOPPER_PEG_Y, 3.0, 3.0, 0.8);
    		robot.intake.intakeStop();
    		delay();
    		robot.dt.setHeading(270, true);
    		dist.setGoal(Constants.NEAR_HOPPER_DEPLOY_X, Constants.NEAR_HOPPER_DEPLOY_Y, 3.0, 4.0, 0.9);
    		delay();
    		robot.shooter.setState(Shooter.Status.STARTED);
    		dist.setGoal(Constants.NEAR_HOPPER_DEPLOY_X, Constants.NEAR_HOPPER_PICKUP_Y, 3.0, 4.0, 0.9);
    		delay();
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
    		
    		//robot.intake.intakeStop();
    		break;
    	case FAR_HOPPER:
    	// Deploy the intakes
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
    		dist.setGoal(52.5/*+50*/, 20, 3.0, 4.0, 0.9);
    		delay();
    		robot.shooter.setState(Shooter.Status.STARTED);
    		robot.sweeper.forwardRoller();
    		robot.sweeper.forwardSweeper();
    		Timer.delay(5);
    		robot.sweeper.stopRoller();
    		robot.sweeper.stopSweeper();
    		Timer.delay(1);
    		robot.shooter.setState(Shooter.Status.OFF);
    		break;
    	case OFF: break;
    	default: break;
    }
    }/**/
    
    public void operatorControl() {
    	dist.disable();
//    	robot.dt.setHeading(0,false);
        while (isOperatorControl() && isEnabled()) {        	
        	controllers.update();
            Timer.delay(0.01);		//10ms Loop Rate
        }
    }
}