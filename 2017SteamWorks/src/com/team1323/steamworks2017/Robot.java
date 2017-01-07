
package com.team1323.steamworks2017;

import ControlSystem.FSM;
import ControlSystem.RoboSystem;
import IO.TeleController;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;


public class Robot extends SampleRobot {
	private RoboSystem robot;
	private TeleController controllers;
	private FSM fsm;
    public Robot() {
        robot = RoboSystem.getInstance();
        controllers = TeleController.getInstance();
        fsm = FSM.getInstance();      
    }
    public void autonomous() {
    	
    }
    
    public void operatorControl() {
    	robot.dt._pidgey.SetFusedHeading(0.0);
        while (isOperatorControl() && isEnabled()) {        	
        	controllers.update();     
            Timer.delay(0.01);		//10ms Loop Rate
        }
    }
}
