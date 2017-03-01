package IO;import ControlSystem.FSM;
import ControlSystem.RoboSystem;
import SubSystems.DistanceController;
import SubSystems.Intake;
import SubSystems.Shooter;
import SubSystems.Swerve;
import SubSystems.Turret;
import Utilities.Constants;
import Utilities.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 
public class TeleController
{
    

    private Controller driver,coDriver;
    private FSM fsm;
    private RoboSystem robot;
    private static TeleController instance = null;
    private boolean robotCentric = false;
    private DistanceController dist;
    
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
    		if(coDriver.aButton.isHeld() && coDriver.aButton.buttonHoldTime() > 1.5){
        		robot.gearIntake.grabGear();
        	}else{
        		robot.gearIntake.extend();
        	}    		
    	}
    	if(coDriver.bButton.isPressed() || coDriver.bButton.isHeld()){
    		robot.gearIntake.retract();
    	}
    	if(coDriver.xButton.isPressed()){
    		robot.gearIntake.scoreGear();
    	}
    	if(coDriver.yButton.isPressed()){
    		
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
    		robot.turret.setState(Turret.State.VisionTracking);
    		robot.gearIntake.retract();
    	}
    	if(coDriver.rightCenterClick.isPressed()){
    		robot.turret.setAngle(0);
    	}
    	if(coDriver.rightTrigger.isHeld()){
    		robot.intake.intakeStop();
    		if(robot.shooter.getStatus() == Shooter.Status.READY){
	    		robot.sweeper.forwardRoller();
	    		robot.sweeper.forwardSweeper();
    		}
    	}else{
    		robot.sweeper.stopRoller();
    		robot.sweeper.stopSweeper();
    	}
    	if(coDriver.leftTrigger.isPressed()){
    		robot.turret.lockAngle(robot.intake.getCurrentAngle());
    		robot.turret.setState(Turret.State.GyroComp);
    		robot.shooter.setGoal(Constants.SHOOTING_SPEED);
    		robot.shooter.setState(Shooter.Status.STARTED);
    	}
    	if(coDriver.startButton.isPressed()){
    		
    	}
    	if(coDriver.getButtonAxis(Controller.RIGHT_STICK_X) > Constants.STICK_DEAD_BAND || coDriver.getButtonAxis(Controller.RIGHT_STICK_X) < -Constants.STICK_DEAD_BAND){
    		robot.turret.setState(Turret.State.Manual);
    		robot.turret.manualControl(coDriver.getButtonAxis(Controller.RIGHT_STICK_X));
    	}
    	
    	if(coDriver.getPOV() == 270)
    		robot.turret.setAngle(-45);
    	if(coDriver.getPOV() == 0)
    		robot.turret.setAngle(0);
    	if(coDriver.getPOV() == 90)
    		robot.turret.setAngle(45);
    	if(coDriver.rightCenterClick.isPressed())
    		robot.turret.setAngle(0);    	
    }
    public void driver() {
    	
    	if(driver.aButton.isPressed()){       
    		robot.dt.setHeading(180,true);
        }
    	if(driver.bButton.isPressed()){  
    		robot.dt.setHeading(90,true);
        }
    	if(driver.bButton.buttonHoldTime() > 2 && driver.bButton.isHeld()){
    		robot.dt.setHeading(120,true);
    	}
    	if(driver.xButton.isPressed()){
    		robot.dt.setHeading(270,true);
        }
    	if(driver.xButton.buttonHoldTime() > 2 && driver.xButton.isHeld()){
    		robot.dt.setHeading(240,true);
    	}
    	if(driver.yButton.isPressed()){
        	robot.dt.setHeading(0,true);
        }
        if(driver.startButton.isHeld()){
        	robot.dt.swerveTrack();
        	robot.dt.sendInput(Util.controlSmoother(driver.getButtonAxis(Xbox.LEFT_STICK_X)), 
            		Util.controlSmoother(driver.getButtonAxis(Xbox.LEFT_STICK_Y)), 
            		Util.turnControlSmoother(0),
            		Util.turnControlSmoother(0),
            		driver.leftTrigger.isHeld(),
            		robotCentric,
            		 driver.rightTrigger.isHeld());
        }else{ 
        	if(!dist.isEnabled()){ 
        		robot.dt.sendInput(Util.controlSmoother(driver.getButtonAxis(Xbox.LEFT_STICK_X)), 
                		Util.controlSmoother(-driver.getButtonAxis(Xbox.LEFT_STICK_Y)), 
                		Util.turnControlSmoother(driver.getButtonAxis(Xbox.RIGHT_STICK_X)),
                		Util.turnControlSmoother(driver.getButtonAxis(Xbox.RIGHT_STICK_Y)),
                		driver.leftTrigger.isHeld(),
                		robotCentric,
                		 driver.rightTrigger.isHeld());
        	}
        
        }
        if(driver.rightTrigger.isPressed()) {
//        	dist.setGoal(0, 10, 0, 4, 0.7); // added this line to help with putting the robot back after testing
        }
        
        if(driver.leftBumper.isPressed()){
//        	robotCentric = false;
        	//dist.setGoal(-30, DistanceController.Direction.X);
//        	dist.setGoal(0,/*-4*/0, 0, 10, 0.5);
        }
        
        if(driver.rightBumper.isPressed()){            
//        	robotCentric = true;
        	//dist.setGoal(30, DistanceController.Direction.X);
//        	dist.setGoal(0, 36, 0, 10, 0.5);
        }
        
        if(driver.backButton.isHeld()){ 
        	robot.intake.setPresetAngles(Intake.AnglePresets.ZERO);
        	robot.dt.resetCoord(Swerve.AnglePresets.ZERO);
        	dist.disable();    
        	robot.dt.setHeading(0, false);
        }
        if(driver.backButton.isPressed()){
//        	robot.shooter.setSpeed(0);
        	    	
        }
        if(driver.startButton.isPressed()){
        	robot.intake.setPresetAngles(Intake.AnglePresets.ONE_EIGHTY);
        	robot.dt.resetCoord(Swerve.AnglePresets.ONE_EIGHTY);
        }
        
        if(driver.rightCenterClick.isPressed()){
//        	robot.shooter.setSpeed(3300); //3475 far shot, 3000 closeshot
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
/**        if(robotCentric)
        	SmartDashboard.putString("RobotControl", "ROBOT");
        else
            SmartDashboard.putString("RobotControl","FIELD");
/**/    }
    public void update(){	
    	driver();
    	coDriver();
    }
    
}
