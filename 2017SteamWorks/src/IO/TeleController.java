package IO;import ControlSystem.FSM;
import ControlSystem.RoboSystem;
import Utilities.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 
public class TeleController
{
    public static final double STICK_DEAD_BAND = 0.2;

    private Controller driver,coDriver;
    private FSM fsm;
    private RoboSystem robot;
    private static TeleController instance = null;
    private boolean robotCentric = false;
    
    public TeleController(){
        driver = new Controller(0);
        driver.start();
        coDriver = new Controller(1);
        coDriver.start();
        robot = RoboSystem.getInstance();
        fsm = FSM.getInstance();
    }
    public static TeleController getInstance(){
        if(instance == null){
            instance = new TeleController();
        }
        return instance;
    }    
   
    public void coDriver(){
    	if(coDriver.xButton.isPressed()){
    		robot.sweeper.forwardRoller();
    		robot.sweeper.forwardSweeper();
    	}
    	if(coDriver.rightBumper.isPressed()){
    		robot.intake.intakeForward();
    	}
    	if(coDriver.leftBumper.isPressed()){
    		robot.intake.intakeReverse();
    	}
    	if(coDriver.backButton.isPressed()){
    		robot.sweeper.stopRoller();
    		robot.sweeper.stopSweeper();
    		robot.intake.intakeStop();
    		robot.shooter.setSpeed(0);
    	}
    	if(coDriver.rightTrigger.isPressed()){
    		robot.shooter.setSpeed(0.75);
    	}
    	if(coDriver.getButtonAxis(Controller.RIGHT_STICK_X) > STICK_DEAD_BAND || coDriver.getButtonAxis(Controller.RIGHT_STICK_X) < -STICK_DEAD_BAND){
    		robot.turret.moveMotor(coDriver.getButtonAxis(Controller.RIGHT_STICK_X));
    	}else{
    		robot.turret.moveMotor(0.0);
    	}
    }
    public void driver() {
    	
    	if(driver.aButton.isPressed()){       
    		robot.dt.setHeading(180);
        }
    	if(driver.bButton.isPressed()){  
    		robot.dt.setHeading(90);
        }
    	if(driver.xButton.isPressed()){
    		robot.dt.setHeading(270);
        }if(driver.yButton.isPressed()){
        	robot.dt.setHeading(0);
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
        robot.dt.sendInput(Util.controlSmoother(driver.getButtonAxis(Xbox.LEFT_STICK_X)), 
        		Util.controlSmoother(-driver.getButtonAxis(Xbox.LEFT_STICK_Y)), 
        		Util.turnControlSmoother(driver.getButtonAxis(Xbox.RIGHT_STICK_X)),
        		Util.turnControlSmoother(driver.getButtonAxis(Xbox.RIGHT_STICK_Y)),
        		driver.leftTrigger.isHeld(),
        		robotCentric,
        		 driver.rightTrigger.isHeld());
        }
        if(driver.rightTrigger.isPressed()) {
        	
        }
        
        if(driver.leftBumper.isPressed()){
        	robotCentric = false; 
        }
        
        if(driver.rightBumper.isPressed()){            
        	robotCentric = true;
        }
        
        if(driver.backButton.isHeld()){ 
        	robot.intake._pidgey.SetFusedHeading(0.0);
        	robot.dt.setHeading(0.0);
        	
        }
        if(driver.backButton.isPressed()){
//        	robot.shooter.setSpeed(0);
        	robot.dt.resetCoord();
        }
        if(driver.startButton.isPressed()){
        	
        }
        
        if(driver.rightCenterClick.isPressed()){
//        	robot.shooter.setSpeed(3300); //3475 far shot, 3000 closeshot
        }
        if(driver.getPOV() == 0){
//        	robot.shooter.bumpUp(100);
        }
        if(driver.getPOV() == 180){
//        	robot.shooter.bumpUp(100);
        }
        if(robotCentric)
        	SmartDashboard.putString("RobotControl", "ROBOT");
        else
            SmartDashboard.putString("RobotControl","FIELD");
    }
    public void update(){	
    	driver();
    	coDriver();
    }
    
}
