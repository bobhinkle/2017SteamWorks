package IO;import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import ControlSystem.FSM;
import ControlSystem.RoboSystem;
import Helpers.MotionProfileExample;
import SubSystems.DistanceController;
import SubSystems.GearIntake;
import SubSystems.Hanger;
import SubSystems.Shooter;
import SubSystems.Swerve;
import SubSystems.Turret;
import Utilities.Constants;
import Utilities.Util;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
 
public class TeleController
{
    
	
    public Controller driver,coDriver;
    private FSM fsm;
    private RoboSystem robot;
    MotionProfileExample _example;
    private static TeleController instance = null;
    private boolean robotCentric = false;
    private DistanceController dist;
    private gearPickedUp gearVibrate;
    private boolean vibrateRunning = false;
    private boolean isBumpedUp = false;
    private boolean isBumpedDown = false;
    private boolean isHanging = false;
    private boolean canReverseSweeper = true;
    private boolean MP_MODE = false;
    public TeleController(){
        driver = new Controller(0);
        driver.start();
        coDriver = new Controller(1);
        coDriver.start();
        robot = RoboSystem.getInstance();
        fsm = FSM.getInstance();
        dist = DistanceController.getInstance();
        _example = new MotionProfileExample(robot.dt.frontRight.driveMotor);
    }
    public static TeleController getInstance(){
        if(instance == null){
            instance = new TeleController();
        }
        return instance;
    }    
 
    public void coDriver(){
    	if(coDriver.aButton.isPressed() || coDriver.aButton.isHeld()){
    		//if(robot.gearIntake.getState() != GearIntake.State.GEAR_DETECTED){
	    		robot.gearIntake.grabGear();
	    		
	    		//robot.gearIntake.extend();	
    		//}
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
    		if(robot.gearIntake.getState() != GearIntake.State.INTAKE_RETRACTED && robot.gearIntake.getState() != GearIntake.State.INTAKE_RETRACTED_WITH_GEAR && robot.gearIntake.getState() != GearIntake.State.GEAR_LOST_RETRACTED)
    			robot.gearIntake.setState(GearIntake.State.INTAKE_EXTENDED_OFF);
    		robot.hanger.setState(Hanger.State.OFF);
    		//robot.hanger.stop();
    		robot.deployBallFlap();
    		robot.dt.stopHanging();
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
    		if(robot.turret.getCurrentState() == Turret.State.VisionTracking){
    			robot.turret.lockAngle(robot.intake.getCurrentAngle(),robot.turret.getAngle());
    			robot.turret.setState(Turret.State.GyroComp);
    		}
    		robot.shooter.setGoal(Constants.SHOOTING_SPEED);
    		robot.shooter.setState(Shooter.Status.STARTED);
    	}
    	if(coDriver.startButton.isPressed()){
    		robot.hanger.setState(Hanger.State.HANGING);
    		//robot.hanger.on();
    		robot.retractBallFlap();
    		robot.dt.startHanging();
    		robot.gearIntake.retract();
    	}
    	if(coDriver.getButtonAxis(Controller.RIGHT_STICK_X) > Constants.STICK_DEAD_BAND || coDriver.getButtonAxis(Controller.RIGHT_STICK_X) < -Constants.STICK_DEAD_BAND){
    		robot.turret.setState(Turret.State.Manual);
    		robot.turret.manualControl(coDriver.getButtonAxis(Controller.RIGHT_STICK_X));
    	}
    	
    	if(coDriver.getPOV() == 270){
    		robot.turret.setState(Turret.State.Off);
    		robot.turret.setAngle(-90);
    	}
    	if(coDriver.getPOV() == 0){
    		robot.turret.setState(Turret.State.Off);
    		robot.turret.setAngle(0);
    	}
    	if(coDriver.getPOV() == 90){
    		robot.turret.setState(Turret.State.Off);
    		robot.turret.setAngle(90);
    	}
    	if(coDriver.getPOV() == 180){
    		if(robot.TeamColor == 1){
    		robot.turret.lockAngle(-180,100);
    		}else{
    			robot.turret.lockAngle(-180,80);
    		}
    		robot.turret.setState(Turret.State.TeleopGyroComp);
    	}
    	if(coDriver.rightCenterClick.isPressed()){
    		robot.turret.setState(Turret.State.Off);
    		robot.turret.setAngle(90);  
    	}
    	if(coDriver.leftCenterClick.isPressed()){
    		robot.turret.setState(Turret.State.Off);
    		robot.turret.setAngle(90);  
    	}
    }
    
    public void driver() {
    	
    	if(driver.aButton.isPressed() || driver.aButton.isHeld()){       
    		robot.dt.setHeading(180,true);
    		robot.dt.enableRotation();
        }else if(driver.bButton.isPressed() || driver.bButton.isHeld()){  
    		robot.dt.setHeading(90,true); //90 degrees
    		robot.dt.enableRotation();
        }else if(driver.xButton.isPressed() || driver.xButton.isHeld()){
    		robot.dt.setHeading(270,true);
    		robot.dt.enableRotation();
        }else if(driver.yButton.isPressed() || driver.yButton.isHeld()){
        	robot.dt.setHeading(0,true);
        	robot.dt.enableRotation();
        }else{
        	robot.dt.disableRotation();
        }
    	
    	if(driver.rightTrigger.isPressed()){
    		robot.gearIntake.scoreGear();
    	}
        if(driver.startButton.isPressed()){
        	robot.hanger.setState(Hanger.State.HANGING);
        	//robot.hanger.on();
        	robot.retractBallFlap();
        	robot.dt.startHanging();
        	robot.gearIntake.retract();
        }
        if(!dist.isEnabled() && !MP_MODE){ 
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
        else if(MP_MODE){
        	setupMP();
        }
        if(driver.leftBumper.isPressed()){
        	//setupMP();
        	//startMP();
        	//MP_MODE = true;
        }
        
        if(driver.rightBumper.isPressed()){            
        	//robot.dt.startMP();
        	//resetToVoltage();
        	//MP_MODE = false;
        }
        
        if(driver.backButton.isHeld()){ 
        	robot.intake.setPresetAngles(0);
        	robot.dt.resetCoord(Swerve.AnglePresets.ZERO);
        	//dist.disable();    
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
        	}
        }
    public void update(){	
    	_example.control();
    	driver();
    	coDriver();
    	if(robot.gearIntake.getState() == GearIntake.State.GEAR_DETECTED){
    		startVibration(0);
    	}else if(robot.turret.onTarget() && robot.turret.getCurrentState() == Turret.State.VisionTracking && fsm.getTargetVisibility()){
    		startVibration(2);	// Uncomment for Roger
    	}else{
    		startVibration(-1);
    	}
    	
    }
    
    
	public void setupMP(){
		//robot.dt.frontRight.driveMotor.reverseSensor(false);
		
		robot.dt.zeroWheels();
		
		robot.dt.frontRight.driveMotor.changeControlMode(TalonControlMode.MotionProfile);
		
		robot.dt.frontRight.driveMotor.setF(0.5842);
		
		robot.dt.frontRight.driveMotor.setP(0.0);
		
		robot.dt.frontRight.driveMotor.setI(0.0);
		
		robot.dt.frontRight.driveMotor.setD(0.0);
		
		CANTalon.SetValueMotionProfile setOutput = _example.getSetValue();
				
		robot.dt.frontRight.driveMotor.set(setOutput.value);
		
		robot.dt.enslaveMotors();

	}
	public void startMP(){
		_example.startMotionProfile();
	}
	public void resetToVoltage(){
		robot.dt.resetModules();
		_example.reset();
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
