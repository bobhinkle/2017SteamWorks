package ControlSystem;

import SubSystems.Navigation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FSM {

	public enum State{
    	DEFAULT, INIT
    }
	private RoboSystem robot;
	private static FSM instance = null;
	public partsUpdate pu;
	
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
//        robot.nav.updatePosition();
//        robot.shooter.leftShooter.update();
//        robot.dt.frontLeft.updateCoord();
    }
}