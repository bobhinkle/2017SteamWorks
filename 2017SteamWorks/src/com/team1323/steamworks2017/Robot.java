
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
/*	TODO The following two lines need to appear in the auto subroutines because they
 * 			each need to set their own start headings. Right?
 * */
        robot.intake._pidgey.SetFusedHeading(90);
        robot.dt.resetCoord();
//        robot.intake._pidgey.
/*        SmartDashboard.putBoolean("Manual Wheel Headings?", true);
		SmartDashboard.putNumber("Manual Heading 1", 0); 
        SmartDashboard.putNumber("Manual Heading 2", 0);
        SmartDashboard.putNumber("Manual Heading 3", 0);
        SmartDashboard.putNumber("Manual Heading 4", 0);
/**/        
    }
    

    
    public void autonomous() {
    	String autoSelected = (String) autoSelect.getSelected();
//    	robot.intake._pidgey.SetFusedHeading(180.0);
//    	robot.dt.resetCoord();
//    	robot.dt.setHeading(0.0);
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
    	robot.dt.resetCoord();
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
    	case TWO_GEAR:
 //   		setStartHeading(180);
    		// intake does something to get the gear?
        	dist.setGoal(0, 57, 0.2, 1.5, 0.8);
    		delay();
    		// reverse gear intake to score
    		// stop gear intake
    		dist.setGoal(0, 30, 0.2, 1.2, 0.8);
    		delay();
    		rotate(90);
   		// move gear intake down
    		// turn gear intake on
    		dist.setGoal(-20, 30, 0.2, 1.0, 0.9);
    		delay();
    		// gear is taken in here
    		// turn gear intake off?
    		// move gear intake up
    		dist.setGoal(0, 30, 0.2, 1.0, 0.9);
    		delay();
    		rotate(180);
    		dist.setGoal(0, 57, 0.2, 1.5, 0.8);
    		delay();
    		// reverse gear intake to score
    		// stop gear intake
    		
    		while(isAutonomous()){Timer.delay(1);}
    		break;
    	case NEAR_HOPPER:
//    		setStartHeading(270);
    		//Move in a positive y direction towards the gear peg
    		dist.setGoal(0, 50, 2.0, 5.0, 0.7); // (0,102)
    		delay();
    		//Move right just a bit to place the gear on the peg
    		dist.setGoal(20, 50, 2.0,5.0, 0.7); // (15,102)
    		delay();
//    		//Turn on the intake so as to collect as many balls as possible throughout autonomous
//    		robot.intake.intakeForward();
    		//Move forward and reach the hopper, deploying it in the process
    		dist.setGoal(-10, 50, 2.0, 5.0, 0.7); //(-15,102)
    		delay();
    		//Move in a positive x direction to fall in the path of the hopper's balls
    		dist.setGoal(-10, 70, 2.0, 5.0, 0.7); //(-15,110)
    		delay();
    		while(isAutonomous()){Timer.delay(1);}
//    		robot.intake.intakeStop();
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
