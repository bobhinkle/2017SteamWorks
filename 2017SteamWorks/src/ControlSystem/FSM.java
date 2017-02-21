package ControlSystem;

import SubSystems.DistanceController;
import Utilities.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FSM {

	public enum State{
    	DEFAULT, INIT,SHOOTER_STARTED,SHOOTER_WAITING,SHOOTER_READY,SHOOTER_SHOOTING, GEAR_GRAB, GEAR_SCORE, INTAKE_BALLS
    }
	private RoboSystem robot;
	private static FSM instance = null;
	public partsUpdate pu;
	private DistanceController dist;
	private State currentState = State.DEFAULT;
    public static FSM getInstance()
    {
        if( instance == null )
            instance = new FSM();
        return instance;
    }
        
    public FSM() {
    	SmartDashboard.putString("FSM", "STARTED");
        robot = RoboSystem.getInstance();
        pu = new partsUpdate();
    	pu.start();
    	dist = DistanceController.getInstance();
    }
   
    public class partsUpdate extends Thread{
        private boolean keepRunning = true;
    	public void run(){
    		SmartDashboard.putString("FSM", "THREAD STARTED");
    		while(keepRunning){
				update();
				Timer.delay(0.01); //10ms Loop Rate
    		} 
        }
        public void kill(){
        	keepRunning = false;
        }
    }

    public void update(){ 
//    	robot.vision.update();
        robot.dt.update();
        robot.intake.update();
        robot.sweeper.SweeperDebug();
        robot.turret.update();
        dist.update();
        robot.gearIntake.update();
        robot.shooter.update();
        
        switch(currentState){
        
        
        }
    }
}