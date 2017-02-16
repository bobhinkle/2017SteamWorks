package IO;

import java.util.Timer;
import java.util.TimerTask;

import Utilities.Util;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Controller extends Joystick
{
    private static final double PRESS_THRESHHOLD = 0.4;
    public static final double DEAD_BAND = 0.15;
    private final Timer mTimer = new Timer();
    private static final int K_READING_RATE = 10;
    private boolean rumbling = false;
    public buttonCheck aButton;
    public buttonCheck bButton;
    public buttonCheck xButton;
    public buttonCheck yButton;
    public buttonCheck startButton;
    public buttonCheck backButton;
    public buttonCheck leftTrigger;
    public buttonCheck rightTrigger;
    public buttonCheck leftBumper;
    public buttonCheck rightBumper;
    public buttonCheck leftCenterClick;
    public buttonCheck rightCenterClick;
    private RumbleThread rumbler;
    public static final int A_BUTTON = 0;
    public static final int B_BUTTON = 1;
    public static final int X_BUTTON = 2;
    public static final int Y_BUTTON = 3;
    public static final int LEFT_BUMPER = 4;
    public static final int RIGHT_BUMPER = 5;
    public static final int BACK_BUTTON = 6;
    public static final int START_BUTTON = 7;
    public static final int LEFT_STICK_Y = 8;
    public static final int LEFT_STICK_X = 9;
    public static final int RIGHT_STICK_Y = 10;
    public static final int RIGHT_STICK_X = 11;
    public static final int LEFT_CENTER_CLICK = 12;
    public static final int RIGHT_CENTER_CLICK = 13;
    public static final int LEFT_TRIGGER = 14;
    public static final int RIGHT_TRIGGER = 15;
    public static final int DPADX = 16;
    public static final int DPADY = 17;
    
    public Controller(int usb)   { 
    	super(usb);
   }
    public void init(){
    	aButton = new buttonCheck(A_BUTTON);
        bButton = new buttonCheck(B_BUTTON);
        xButton = new buttonCheck(X_BUTTON);
        yButton = new buttonCheck(Y_BUTTON);
        startButton = new buttonCheck(START_BUTTON);
        backButton = new buttonCheck(BACK_BUTTON);
        leftTrigger = new buttonCheck(LEFT_TRIGGER);
        rightTrigger = new buttonCheck(RIGHT_TRIGGER);
        leftBumper = new buttonCheck(LEFT_BUMPER);
        rightBumper = new buttonCheck(RIGHT_BUMPER);
        leftCenterClick = new buttonCheck(LEFT_CENTER_CLICK);
        rightCenterClick = new buttonCheck(RIGHT_CENTER_CLICK);        
    }
    public void start() {
        synchronized (mTimer) {
            mTimer.schedule(new InitTask(), 0);
        }
    }
	private class InitTask extends TimerTask {
        @Override
        public void run() {
            while (true) {
                try {
                	SmartDashboard.putString("FSM", "STARTED");
                	init();
                    break;
                } catch (Exception e) {
                    System.out.println("FSM failed to initialize: " + e.getMessage());
                    synchronized (mTimer) {
                        mTimer.schedule(new InitTask(), 500);
                    }
                }
            }
            synchronized (mTimer) {
                mTimer.schedule(new UpdateTask(), 0, (int) (1000.0 / K_READING_RATE));
            }
        }
    }
    private boolean getAButton()      { return getRawButton(1); }
    private boolean getBButton()      { return getRawButton(2); }
    private boolean getXButton()      { return getRawButton(3); }
    private boolean getYButton()      { return getRawButton(4); }
    private boolean getLeftBumper()   { return getRawButton(5); }
    private boolean getRightBumper()  { return getRawButton(6); }
    private boolean getBackButton()   { return getRawButton(7); }
    private boolean getStartButton()  { return getRawButton(8); }
    private double  getLeftStickX()   { return getRawAxis(0); }
    private double  getLeftStickY()   { return getRawAxis(1); }
    private double  getRightStickX()  { return getRawAxis(4); }
    private double  getRightStickY()  { return getRawAxis(5); }
    private double  getDPADX()        { return getRawAxis(6); }
    private double  getDPADY()        { return getRawAxis(7); }
    private boolean getLeftStick()    { return getRawButton(9); }
    private boolean getRightStick()   { return getRawButton(10); }
    private boolean getRightTrigger() { return getRawAxis(3) > PRESS_THRESHHOLD;}
    private boolean getLeftTrigger()  { return getRawAxis(2) > PRESS_THRESHHOLD; }
    
    public double getButtonAxis(int button){
    	switch(button){
    	case LEFT_STICK_X:
    		return Util.deadBand(getLeftStickX(), DEAD_BAND);
    	case LEFT_STICK_Y:
    		return Util.deadBand(getLeftStickY(),DEAD_BAND);
    	case RIGHT_STICK_X:
    		return Util.deadBand(getRightStickX(),DEAD_BAND);
    	case RIGHT_STICK_Y:
    		return Util.deadBand(getRightStickY(),DEAD_BAND);
    	case DPADX:
    		return Util.deadBand(getDPADX(),DEAD_BAND);
    	case DPADY:
    		return Util.deadBand(getDPADY(),DEAD_BAND);
    	default:
    		return 0;
    	}
    }
    public enum Vibration{
    	SINGLE, DOUBLE,PULSE
    }
    public void rumble(Vibration _mode){
    	if(!rumbling){
	    	rumbler = new RumbleThread();
	    	rumbler.setVibration(_mode);
	    	rumbler.start();
    	}
    }
    public class RumbleThread extends Thread{
    	private Vibration mode;
		@Override
		public void run() {
			rumbling = true;
			switch(mode){
	    	case SINGLE:	    		
	    		try {	    			
	    			setRumble(RumbleType.kLeftRumble,1);
		    		setRumble(RumbleType.kRightRumble,1);
					sleep(500);
					setRumble(RumbleType.kLeftRumble,0);
		    		setRumble(RumbleType.kRightRumble,0);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					rumbling = false;
					e.printStackTrace();
				}
	    		break;
	    	case DOUBLE:
	    		try {
	    			setRumble(RumbleType.kLeftRumble,1);
		    		setRumble(RumbleType.kRightRumble,1);
					sleep(250);
					setRumble(RumbleType.kLeftRumble,0);
		    		setRumble(RumbleType.kRightRumble,0);
		    		sleep(250);
		    		setRumble(RumbleType.kLeftRumble,1);
		    		setRumble(RumbleType.kRightRumble,1);
					sleep(250);
					setRumble(RumbleType.kLeftRumble,0);
		    		setRumble(RumbleType.kRightRumble,0);
		    		sleep(500);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
					rumbling = false;
				}
	    		break;
	    	case PULSE:
	    		
	    		break;
	    	}
			rumbling = false;
		}
		public void setVibration(Vibration _mode){
	    	mode = _mode;
	    }
    }
    public class buttonCheck{
    	private int buttonState = 0;
    	private double buttonStartTime = 0;
    	private boolean buttonCheck = false;;
    	private int buttonNumber = 0;
    	private static final int NOT_PRESSED = 0;
    	private static final int FIRST_PRESS = 1;
    	private static final int HELD        = 2;
    	private boolean released = false;
    	public buttonCheck(int number){
    		buttonNumber = number;
    		buttonState = NOT_PRESSED;
    	}
    	public boolean isReleased(){
    		if(released){
    			released = false;
    			return true;
    		}return false;
    	}
    	public boolean isPressed(){
    		return buttonState == FIRST_PRESS;
    	}
    	public double buttonHoldTime(){
    		if(buttonState == HELD)
    			return System.currentTimeMillis()-buttonStartTime;
    		return 0;
    	}
    	public boolean isHeld(){
    		return buttonState == HELD;
    	}
    	public void update(){
    		switch(buttonNumber){
    		case A_BUTTON:
    			buttonCheck = getAButton();
    			break;
    		case B_BUTTON:
    			buttonCheck = getBButton();
    			break;
    		case X_BUTTON:
    			buttonCheck = getXButton();
    			break;
    		case Y_BUTTON:
    			buttonCheck = getYButton();
    			break;
    		case START_BUTTON:
    			buttonCheck = getStartButton();
    			break;
    		case BACK_BUTTON:
    			buttonCheck = getBackButton();
    			break;
    		case LEFT_BUMPER:
    			buttonCheck = getLeftBumper();
    			break;
    		case RIGHT_BUMPER:
    			buttonCheck = getRightBumper();
    			break;
    		case LEFT_TRIGGER:
    			buttonCheck = getLeftTrigger();
    			break;
    		case RIGHT_TRIGGER:
    			buttonCheck = getRightTrigger();
    			break;
    		case LEFT_CENTER_CLICK:
    			buttonCheck = getLeftStick();
    			break;
    		case RIGHT_CENTER_CLICK:
    			buttonCheck = getRightStick();
    			break;
    		}
    		if(buttonCheck){
    			switch(buttonState){
    			case NOT_PRESSED:
    				buttonState = FIRST_PRESS;
    				buttonStartTime = System.currentTimeMillis();
    				break;
    			case FIRST_PRESS:
    				buttonState = HELD;
    				break;
    			}
    		}else{
    			if(buttonState == FIRST_PRESS || buttonState == HELD){
    				released = true;
    			}
    			buttonState = NOT_PRESSED;
    		}
    	}
    }
    private class UpdateTask extends TimerTask {
	    public void run(){ 
	    	aButton.update();
	    	bButton.update();
	    	xButton.update();
	    	yButton.update();
	    	startButton.update();
	    	backButton.update();
	    	leftTrigger.update();
	    	rightTrigger.update();
	    	leftBumper.update();
	    	rightBumper.update();
	    	leftCenterClick.update();
	    	rightCenterClick.update();
	    }
    }
}