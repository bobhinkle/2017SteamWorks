package IO;import ControlSystem.FSM;
import ControlSystem.RoboSystem;
import SubSystems.Vision;
import Utilities.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 
public class TeleController
{
    public static final double STICK_DEAD_BAND = 0.2;

    private Xbox driver;
    private FSM fsm;
    private RoboSystem robot;
    private static TeleController instance = null;
    private boolean robotCentric = false;
    
    public TeleController(){
        driver = new Xbox(0);
        driver.init();
        robot = RoboSystem.getInstance();
        fsm = FSM.getInstance();
    }
    public static TeleController getInstance(){
        if(instance == null){
            instance = new TeleController();
        }
        return instance;
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
            		Util.controlSmoother(-driver.getButtonAxis(Xbox.LEFT_STICK_Y)), 
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
        	robot.dt._pidgey.SetFusedHeading(0.0);
        	robot.dt.setHeading(0.0);
        }
        
        if(driver.startButton.isPressed()){
        	
        }
        
        if(driver.rightCenterClick.isPressed()){
        	
        }
        if(driver.getPOV() == 0){
        	
        }
        if(robotCentric)
        	SmartDashboard.putString("RobotControl", "ROBOT");
        else
            SmartDashboard.putString("RobotControl","FIELD");
    }
    public void update(){
    	driver.run();
    	driver();
    }
    
}
