
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
	final String one_gear    = "one_gear";
	final String two_gear    = "two_gear";
	final String near_hopper = "near_hopper";
	SendableChooser autoSelect;
	private DistanceController dist;
	public static enum AUTO{
    	OFF,ONE_GEAR,TWO_GEAR,NEAR_HOPPER
    }
    public Robot() {
    	autoSelect = new SendableChooser();
        autoSelect.addDefault("Off", defaultAuto);
        autoSelect.addObject("One_Gear", one_gear);
        autoSelect.addObject("Two_Gear", two_gear);
        autoSelect.addObject("Near_Hopper", near_hopper);
        SmartDashboard.putData("Select Auto", autoSelect);  
        robot = RoboSystem.getInstance();
        controllers = TeleController.getInstance();
        fsm = FSM.getInstance(); 
        dist = DistanceController.getInstance();

        SmartDashboard.putBoolean("Manual Wheel Headings?", true);
		SmartDashboard.putNumber("Manual Heading 1", 0); 
        SmartDashboard.putNumber("Manual Heading 2", 0);
        SmartDashboard.putNumber("Manual Heading 3", 0);
        SmartDashboard.putNumber("Manual Heading 4", 0);
        
    }
    

    
    public void autonomous() {
    	String autoSelected = (String) autoSelect.getSelected();
    	robot.intake._pidgey.SetFusedHeading(0.0);
    	robot.dt.setHeading(0.0);
    		switch(autoSelected){
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
    private void setStartHeading(double angle) {
    	robot.intake._pidgey.SetFusedHeading(angle);
//    	robot.dt.setHeading(angle);
    }

    public void rotate(double angle){
    	int timeout = 0;
    	robot.dt.setHeading(angle);
		while(!robot.dt.headingOnTarget() && isAutonomous() && (timeout < 200)){
			robot.dt.sendInput(0, 0, 0, 0, false, true, false);
			Timer.delay(0.01);
			timeout++;
		}
    }
    
    public void executeAuto(AUTO autoSelect){
    	    	
    	switch(autoSelect){
    	case TWO_GEAR:
//        	robot.intake._pidgey.SetFusedHeading(180.0);
//        	robot.dt.setHeading(180.0);
    		//Place the first gear
    		dist.setGoal(robot.dt.frontLeft.getX(),-66, 2.0,5.0, 0.7);
    		//Move back to pick up gear
    		dist.setGoal(robot.dt.frontLeft.getX(),55, 2.0,5.0, 0.7);
    		//Rotate to face second gear
    		rotate(90);
    		//Move forward a bit to acquire the gear
    		dist.setGoal(robot.dt.frontLeft.getX(),-10, 2.0,5.0, 0.7);
    		//Move back to original x position
    		dist.setGoal(robot.dt.frontLeft.getX(),10, 2.0,5.0, 0.7);
    		//Rotate back to face the gear peg
    		rotate(180);
    		//Move forward to place the second gear
    		dist.setGoal(robot.dt.frontLeft.getX(),-66, 2.0,5.0, 0.7);
    		break;
    	case NEAR_HOPPER:
    		//Move in a positive x direction towards the gear peg
    		dist.setGoal(robot.dt.frontLeft.getX(),97, 2.0,5.0, 0.7);
    		//Move backward just a bit to place the gear on the peg
    		dist.setGoal(robot.dt.frontLeft.getX(),-5, 2.0,5.0, 0.7);
    		//Turn on the intake so as to collect as many balls as possible throughout autonomous
    		robot.intake.intakeForward();
    		//Move forward and reach the hopper, deploying it in the process
    		dist.setGoal(robot.dt.frontLeft.getX(),120, 2.0,5.0, 0.7);
    		//Move in a positive x direction to fall in the path of the hopper's balls
    		dist.setGoal(robot.dt.frontLeft.getX(),5, 2.0,5.0, 0.7);
    		
    		robot.intake.intakeStop();
    		break;
    	case OFF: break;
    	default: break;
    }
    }/**/
    
    public void operatorControl() {
    	dist.disable();
    	robot.intake._pidgey.SetFusedHeading(0.0);
    	Timer.delay(0.2); 
    	robot.dt.setHeading(0);
        while (isOperatorControl() && isEnabled()) {        	
        	controllers.update();
        	robot.intake.pigeonUpdate();
            Timer.delay(0.01);		//10ms Loop Rate
        }
    }
}
