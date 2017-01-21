
package com.team1323.steamworks2017;

import ControlSystem.FSM;
import ControlSystem.RoboSystem;
import IO.TeleController;
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
    }
    public void autonomous() {
    	String autoSelected = (String) autoSelect.getSelected();
    	robot.dt._pidgey.SetFusedHeading(0.0);
    	robot.dt.setHeading(0.0);
    	switch(autoSelected){
    	case one_gear:
    		executeAuto(AUTO.ONE_GEAR);
    		break;
		case defaultAuto:
			executeAuto(AUTO.OFF);
			break;
    	}
    }
    public void executeAuto(AUTO autoSelect){
    	    	
    	switch(autoSelect){
    	case ONE_GEAR:
    		double timeout = System.currentTimeMillis();
    		while(timeout+2000 > System.currentTimeMillis()){
    			robot.dt.sendInput(0.0, 0.5, 0, 0, true, true, false);
    		}
    		timeout = System.currentTimeMillis();
    		while(timeout+1000 > System.currentTimeMillis()){
    			robot.dt.sendInput(0, 0, 0, 0, true, true, false);
    		}
    		timeout = System.currentTimeMillis();
    		while(timeout+2000 > System.currentTimeMillis()){
    			robot.dt.sendInput(0.0, -0.5, 0, 0, true, true, false);
    		}
    		timeout = System.currentTimeMillis();
    		while(timeout+1000 > System.currentTimeMillis()){
    			robot.dt.sendInput(0, 0, 0, 0, true, true, false);
    		}
    		timeout = System.currentTimeMillis();
    		while(timeout+2000 > System.currentTimeMillis()){
    			robot.dt.sendInput(0, 1.0, -0.65, 0, false, false, true);
    		}
    		timeout = System.currentTimeMillis();
    		while(timeout+1000 > System.currentTimeMillis()){
    			robot.dt.sendInput(0, 0, 0, 0, true, true, false);
    		}
    		break;
    		
    	case OFF:
    		
    		break;
    	default:
    		
    		break;
    }
    }
    
    public void operatorControl() {
    	robot.dt._pidgey.SetFusedHeading(0.0);
        while (isOperatorControl() && isEnabled()) {        	
        	controllers.update();     
            Timer.delay(0.01);		//10ms Loop Rate
        }
    }
}
