package IO;import ControlSystem.FSM;
import ControlSystem.RoboSystem;
import SubSystems.DistanceController;
import SubSystems.GearIntake;
import SubSystems.GearIntake.State;
import SubSystems.Hanger;
import SubSystems.Intake;
import SubSystems.Shooter;
import SubSystems.Swerve;
import SubSystems.Turret;
import Utilities.Constants;
import Utilities.Util;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; //added?
 
public class TeleController
{
    

    public Controller driver,coDriver;
    private FSM fsm;
    private RoboSystem robot;
    private static TeleController instance = null;
    private boolean robotCentric = false;
    private DistanceController dist;
    private gearPickedUp gearVibrate;
    private boolean vibrateRunning = false;
    private boolean isBumpedUp = false;
    private boolean isBumpedDown = false;
    private boolean isHanging = false;
    private boolean canReverseSweeper = true;
    public TeleController(){
        driver = new Controller(0);
        driver.start();
        coDriver = new Controller(1);
        coDriver.start();
        robot = RoboSystem.getInstance();
        fsm = FSM.getInstance();
        dist = DistanceController.getInstance();
    }
    public static TeleController getInstance(){
        if(instance == null){
            instance = new TeleController();
        }
        return instance;
    }    
 
    public void coDriver(){
    	if(coDriver.aButton.isPressed() || coDriver.aButton.isHeld()){
    		if(robot.gearIntake.getState() != GearIntake.State.GEAR_DETECTED){
	    		robot.gearIntake.grabGear();
	    		//robot.gearIntake.extend();	
    		}
    	}
    	if(coDriver.bButton.isPressed() || coDriver.bButton.isHeld()){
    		robot.gearIntake.retract();
    	}
    	if(coDriver.xButton.isPressed()){
    		robot.turret.setState(Turret.State.VisionTracking);
    		isBumpedUp = false;
    		isBumpedDown = false;
    	}
    	if(coDriver.yButton.isHeld()){
    		/*robot.turret.lockAngle(robot.intake.getCurrentAngle());
    		robot.turret.setState(Turret.State.GyroComp);
    		robot.shooter.setGoal(robot.shooter.getShooterSpeedForRange(fsm.getTargetDistance()));
    		robot.shooter.setState(Shooter.Status.STARTED);*/
    		//robot.gearIntake.autoDep();
    		robot.sweeper.stopRoller();
    		robot.sweeper.stopSweeper();
    		if(robot.sweeper.isStopped()){
    			robot.sweeper.reverseRoller();
    			robot.sweeper.needsToStop = true;
    		}
    	}else{
    		if(robot.sweeper.needsToStop == true){
    			robot.sweeper.stopSweeper();
    			robot.sweeper.stopRoller();
    			robot.sweeper.needsToStop = false;
    		}
    	}
    	if(coDriver.rightBumper.isPressed()){
    		robot.intake.intakeForward();
    	}
    	if(coDriver.leftBumper.isPressed()){
    		robot.intake.intakeReverse();
    	}
    	if(coDriver.backButton.isPressed()){
    		robot.intake.intakeStop();
    		robot.shooter.stop();  		
    		robot.sweeper.stopRoller();
    		robot.sweeper.stopSweeper();
    		canReverseSweeper = true;
    		robot.turret.setState(Turret.State.VisionTracking);
    		if(robot.gearIntake.getState() != GearIntake.State.INTAKE_RETRACTED && robot.gearIntake.getState() != GearIntake.State.INTAKE_RETRACTED_WITH_GEAR && robot.gearIntake.getState() != GearIntake.State.GEAR_LOST_RETRACTED)
    			robot.gearIntake.setState(GearIntake.State.INTAKE_EXTENDED_OFF);
    		robot.hanger.stop();
    	}
    	if(coDriver.rightCenterClick.isPressed()){
    		robot.turret.setAngle(0);
    	}
    	if(coDriver.rightTrigger.isPressed()){
    		if(robot.shooter.getStatus() == Shooter.Status.READY){
	    		robot.intake.intakeStop();
	    		robot.sweeper.turnSweeperOn();
		    	canReverseSweeper = false;
    		}
    	}
    	if(coDriver.leftTrigger.isPressed()){
    		robot.turret.lockAngle(robot.intake.getCurrentAngle());
    		robot.turret.setState(Turret.State.GyroComp);
    		robot.shooter.setGoal(robot.shooter.getShooterSpeedForRange(fsm.getTargetDistance()));
    		robot.shooter.setState(Shooter.Status.STARTED);
    	}
    	if(coDriver.startButton.isPressed()){
    		robot.hanger.setState(Hanger.State.HANGING);
    	}
    	if(coDriver.getButtonAxis(Controller.RIGHT_STICK_X) > Constants.STICK_DEAD_BAND || coDriver.getButtonAxis(Controller.RIGHT_STICK_X) < -Constants.STICK_DEAD_BAND){
    		robot.turret.setState(Turret.State.Manual);
    		robot.turret.manualControl(coDriver.getButtonAxis(Controller.RIGHT_STICK_X));
    	}
    	
    	if(coDriver.getPOV() == 270)
    		robot.turret.setAngle(-90);
    	
    	if(coDriver.getPOV() == 0)
    		robot.gearIntake.forward();
    		//robot.turret.resetAngle(90);
    		//robot.turret.setAngle(0);
    		/*if(!isBumpedUp && robot.shooter.getStatus() != Shooter.Status.OFF){
    			robot.shooter.setGoal(robot.shooter.getTarget() + 100);
    			robot.shooter.setState(Shooter.Status.STARTED);
    			if(isBumpedDown){
    				isBumpedUp = false;
    				isBumpedDown = false;
    			}else{
    				isBumpedUp = true;
    			}
    		}*/
    	if(coDriver.getPOV() == 90)
    		robot.turret.setAngle(90);
    	if(coDriver.getPOV() == 180){
    		robot.gearIntake.reverse();
    		//robot.turret.setAngle(-2);
/*    		if(!isBumpedDown && robot.shooter.getStatus() != Shooter.Status.OFF){
    			robot.shooter.setGoal(robot.shooter.getTarget() - 100);
    			robot.shooter.setState(Shooter.Status.STARTED);
    			if(isBumpedUp){
    				isBumpedDown = false;
    				isBumpedUp=  false;
    			}else{
    				isBumpedDown = true;
    			}
    		}*/
    	}
    	if(coDriver.rightCenterClick.isPressed())
    		robot.turret.setAngle(0);   
    }
    
    public void driver() {
    	
    	if(driver.aButton.isPressed()){       
    		robot.dt.setHeading(90,true);
        }
    	if(driver.bButton.isPressed()){  
    		robot.dt.setHeading(0,true); //90 degrees
        }
    	if(driver.bButton.buttonHoldTime() > 2 && driver.bButton.isHeld()){
    		//robot.dt.setHeading(120,true);
    	}
    	if(driver.xButton.isPressed()){
    		robot.dt.setHeading(180,true);
        }
    	if(driver.xButton.buttonHoldTime() > 2 && driver.xButton.isHeld()){
    		//robot.dt.setHeading(240,true);
    	}
    	if(driver.yButton.isPressed()){
        	robot.dt.setHeading(270,true);
        }
    	if(driver.rightTrigger.isPressed()){
    		robot.gearIntake.scoreGear();
    	}
        if(driver.startButton.isPressed()){
        	robot.hanger.setState(Hanger.State.HANGING);
        }
        if(!dist.isEnabled()){ 
        	/*	robot.dt.sendInput(Util.controlSmoother(driver.getButtonAxis(Xbox.LEFT_STICK_X)), 
                		Util.controlSmoother(-driver.getButtonAxis(Xbox.LEFT_STICK_Y)),0,
                		0,
                		driver.leftTrigger.isHeld(),
                		robotCentric,
                		 false);*/
        		robot.dt.sendInput(Util.controlSmoother(driver.getButtonAxis(Xbox.LEFT_STICK_X)), 
                		Util.controlSmoother(-driver.getButtonAxis(Xbox.LEFT_STICK_Y)), 
                		Util.turnControlSmoother(driver.getButtonAxis(Xbox.RIGHT_STICK_X)),
                		Util.turnControlSmoother(driver.getButtonAxis(Xbox.RIGHT_STICK_Y)),
                		driver.leftTrigger.isHeld() || driver.leftTrigger.isPressed(),
                		robotCentric,
                		 false, driver.leftBumper.isPressed() || driver.leftBumper.isHeld());
        	}
        
        
        if(driver.rightTrigger.isPressed()) {
//        	dist.setGoal(0, 10, 0, 4, 0.7); // added this line to help with putting the robot back after testing
        }
        
        if(driver.leftBumper.isPressed()){
//        	robotCentric = false;
        	//dist.setGoal(-30, DistanceController.Direction.X);
        	//dist.setGoal(0, 0, 1.0, 10, 0.5);
        }
        
        if(driver.rightBumper.isPressed()){            
//        	robotCentric = true;
        	//dist.setGoal(30, DistanceController.Direction.X);
//        	dist.setGoal(0, 36, 0, 10, 0.5);
        }
        
        if(driver.backButton.isHeld()){ 
        	robot.intake.setPresetAngles(0);
        	robot.dt.resetCoord(Swerve.AnglePresets.ZERO);
        	dist.disable();    
        	robot.dt.setHeading(0, false);
        }
        if(driver.backButton.isPressed()){
//        	robot.shooter.setSpeed(0);
        	    	
        }
        /*if(driver.startButton.isPressed()){
        	robot.intake.setPresetAngles(Intake.AnglePresets.ONE_EIGHTY);
        	robot.dt.resetCoord(Swerve.AnglePresets.ONE_EIGHTY);
        }*/
        
        if(driver.rightCenterClick.isPressed()){
//        	robot.shooter.setSpeed(3300); //3475 far shot, 3000 close shot
        }
        if(driver.getPOV() == 0){
//        	robot.shooter.bumpUp(100);
        	// This button will now be used to test the distance controller's accuracy
        	//dist.setGoal(0, 40, 0, 5, 0.5);
        }
        if(driver.getPOV() == 90){
        	
        }
        if(driver.getPOV() == 180){
//        	robot.shooter.bumpUp(100);
        	// This button will now be used to test starting orientations
 //       	robot.intake._pidgey.SetFusedHeading(180.0);
 //       	robot.dt.setHeading(0.0);
 //       	dist.setGoal(0,/*-4*/0, 0, 5, 0.5);
        }
/*        if(robotCentric)
        	SmartDashboard.putString("RobotControl", "ROBOT");
        else
            SmartDashboard.putString("RobotControl","FIELD");
/**/    }
    public void update(){	
    	driver();
    	coDriver();
    	if(robot.gearIntake.getState() == GearIntake.State.GEAR_DETECTED){
    		startVibration(0);
    	}else if(robot.turret.onTarget() && robot.turret.getCurrentState() == Turret.State.VisionTracking && fsm.getTargetVisibility()){
    		//startVibration(2);	// Uncomment for Roger
    	}else{
    		startVibration(-1);
    	}
    	
    }
    
    private void startVibration(int type){
    	if(!vibrateRunning){
	    	gearVibrate = new gearPickedUp();
	    	gearVibrate.setPulseType(type);
	    	gearVibrate.start();
    	}
    }
    public class gearPickedUp extends Thread{
    	private int pulseType = 0;
    	public void setPulseType(int p){
    		pulseType = p;
    	}
    	public void run(){
    		vibrateRunning = true;
    		switch(pulseType){
    		case 0:
    			for(int i = 0; i < 2; i++){
        			coDriver.setRumble(RumbleType.kLeftRumble, 1.0);
        			driver.setRumble(RumbleType.kLeftRumble, 1.0);
            		Timer.delay(0.5);
            		coDriver.setRumble(RumbleType.kLeftRumble, 0.0);
            		driver.setRumble(RumbleType.kLeftRumble, 0.0);
    			}
    			break;
    		case 1:
    			switch(robot.gearIntake.getState()){
	    			case GEAR_LOST_EXTENDED:
	    				//robot.gearIntake.grabGear();
	    				break;
	    			case GEAR_LOST_RETRACTED:
	    				
	    				break;
    			}
    			
    			for(int i = 0; i < 2; i++){
        			coDriver.setRumble(RumbleType.kLeftRumble, 0.75);
        			coDriver.setRumble(RumbleType.kRightRumble, 0.0);
        			driver.setRumble(RumbleType.kLeftRumble, 0.75);
        			driver.setRumble(RumbleType.kRightRumble, 0.0);
            		Timer.delay(0.5);
            		coDriver.setRumble(RumbleType.kLeftRumble, 0.0);
            		coDriver.setRumble(RumbleType.kRightRumble, 0.75);
            		driver.setRumble(RumbleType.kLeftRumble, 0.0);
            		driver.setRumble(RumbleType.kRightRumble, 0.75);
            		Timer.delay(1);
            		coDriver.setRumble(RumbleType.kRightRumble, 0.0);
        			coDriver.setRumble(RumbleType.kLeftRumble, 0.0);
        			driver.setRumble(RumbleType.kRightRumble, 0.0);
        			driver.setRumble(RumbleType.kLeftRumble, 0.0);
        		}    
    			break;
    		case 2:
    			for(int i = 0; i < 2; i++){
        			coDriver.setRumble(RumbleType.kLeftRumble, 0.7);
        			Timer.delay(1);
            		coDriver.setRumble(RumbleType.kLeftRumble, 0.0);
    			}
        		break;
    		case -127:
    			coDriver.setRumble(RumbleType.kRightRumble, 0.0);
    			coDriver.setRumble(RumbleType.kLeftRumble, 0.0);
    			driver.setRumble(RumbleType.kRightRumble, 0.0);
    			driver.setRumble(RumbleType.kLeftRumble, 0.0);
    			break;
    		default:
    			coDriver.setRumble(RumbleType.kRightRumble, 0.0);
    			coDriver.setRumble(RumbleType.kLeftRumble, 0.0);
    			driver.setRumble(RumbleType.kRightRumble, 0.0);
    			driver.setRumble(RumbleType.kLeftRumble, 0.0);
    			break;
    		}
    				
    		vibrateRunning = false;
        }
    }
}
