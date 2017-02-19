
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
	SendableChooser autoSelect;
	private DistanceController dist;
	public static enum AUTO{
    	OFF,ONE_GEAR
    }
    public Robot() {
    	autoSelect = new SendableChooser();
        autoSelect.addDefault("Off", defaultAuto);
        autoSelect.addObject("One_Gear", one_gear);
        SmartDashboard.putData("Auto Select", autoSelect);  
        robot = RoboSystem.getInstance();
        controllers = TeleController.getInstance();
        fsm = FSM.getInstance(); 
        dist = DistanceController.getInstance();
    }
    public void autonomous() {
    	String autoSelected = (String) autoSelect.getSelected();
    	robot.intake._pidgey.SetFusedHeading(0.0);
    	robot.dt.setHeading(0.0);
   		// set to false to disable execution of any auto
    		switch(autoSelected){
    			case one_gear:
    				executeAuto(AUTO.ONE_GEAR);
    				break;
    			case defaultAuto:
    				executeAuto(AUTO.ONE_GEAR);
    				break;
    		}
    }
    
    public void executeAuto(AUTO autoSelect){
    	    	
    	switch(autoSelect){
    	case ONE_GEAR:
    		dist.setGoal(-66, DistanceController.Direction.Y, 2.0);
    		while(!dist.onTarget() && isAutonomous()){
    			Timer.delay(0.01);
    		}
    		dist.setGoal(55, DistanceController.Direction.Y, 4.0);
    		while(!dist.onTarget() && isAutonomous()){
    			Timer.delay(0.01);
    		}
    		robot.dt.setHeading(270);
    		robot.dt.sendInput(0, 0, 0, 0, false, true, false);

    		while(!robot.dt.headingOnTarget() && isAutonomous()){
    			robot.dt.sendInput(0, 0, 0, 0, false, true, false);
    			Timer.delay(0.01);
    		}
    		break;
    		
    		
    	case OFF: break;
    	default: break;
    }
    }/**/
    
    public void operatorControl() {
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
