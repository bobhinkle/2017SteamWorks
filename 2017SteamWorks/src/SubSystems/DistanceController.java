package SubSystems;

import Helpers.SynchronousPID;
import IO.Logger;
import Utilities.Constants;
import Utilities.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DistanceController {

	private static DistanceController instance;
	private double targetY = 0.0;
	private double targetX = 0.0;
	private double allowableErrorX = 1.0;
	private double allowableErrorY = 1.0;
	private boolean onTarget = false;
	private int cyclesOnTarget = 0;
	private int cyclesToCheck = 0;
	private Swerve dt;
	private double inputY = 0.0;
	private double inputX = 0.0;
	private double lastDistanceY = 0.0;
	private double lastDistanceX = 0.0;
	private boolean isEnabled = false;
	private double timeout = 0;
	private Logger logger;
	private double inputCap = 1.0;
	private double minCap  = 0.005;
	private long lastTimeStamp = 0;
	private double yRate = 0.0;
	private SynchronousPID xPID;
	private SynchronousPID yPID;
	private boolean followerNegated = false;
	public enum ControlWheel{
		SWERVE, FOLLOWER
	}
	private ControlWheel yWheel = ControlWheel.FOLLOWER;
	private ControlWheel xWheel = ControlWheel.SWERVE;
	public DistanceController(){
		dt = Swerve.getInstance();
		xPID = new SynchronousPID(Constants.DIST_CONTROLLER_X_P,0.0,Constants.DIST_CONTROLLER_X_D, Constants.DIST_CONTROLLER_X_FF);
		xPID.setOutputRange(-1.0, 1.0);
		yPID = new SynchronousPID(Constants.DIST_CONTROLLER_Y_LONG_P,0.0,Constants.DIST_CONTROLLER_Y_LONG_D, Constants.DIST_CONTROLLER_Y_LONG_FF);
		yPID.setOutputRange(-1.0, 1.0);
		logger = Logger.getInstance();
	}
	public static DistanceController getInstance(){
		if(instance == null)
			instance = new DistanceController();
		return instance;
	}
	private double currentPositionY = 0.0;
	private double currentPositionX = 0.0;
	public void update(){
//172.22.11.2
		SmartDashboard.putBoolean(" Distance Controller Enabled ", isEnabled);
		SmartDashboard.putBoolean(" Dist On Target ", isOnTarget());
		updateCurrentPos();
	
		SmartDashboard.putNumber(" Dist X Set Point ", targetX);
		SmartDashboard.putNumber(" Dist Y Set Point ", targetY);
		SmartDashboard.putNumber(" Dist Y Position ", getYInches());
		SmartDashboard.putNumber(" Dist Y Error ", getYError());
		SmartDashboard.putNumber(" Dist X Position ", getXInches());
		SmartDashboard.putNumber(" Dist X Error ", getXError());
		SmartDashboard.putNumber("distInputY", yPID.get());
		SmartDashboard.putNumber("distInputX", xPID.get()); 
		SmartDashboard.putNumber("Distance travelled", yRate);
		
		if(isEnabled){
			updateCurrentPos();
			xPID.calculate(currentPositionX);
			yPID.calculate(currentPositionY);
			if(timeout >= System.currentTimeMillis()){
				if(!isOnTarget()){
					cyclesOnTarget = cyclesToCheck;	
					//System.out.println("X input: " + Double.toString(xPID.get()) + " Y Input: " + Double.toString(yPID.get()));
					dt.sendInput(xPID.get(), yPID.get(), 0.0, 0.0, false, false, false, false);
				}else{
					dt.sendInput(0, 0, 0, 0, false, false, false, false); // second false was true					
					if(cyclesOnTarget <= 0){						
						logger.writeToLog("DIST Reached Target: " + Double.toString(timeout-System.currentTimeMillis()) + " ms to spare");
						onTarget = true;
						disable();						
					}else{
						cyclesOnTarget--;
					}
					
				}
			}else{
				dt.sendInput(0, 0, 0, 0, false, false, false, false);
				onTarget = true;
				logger.writeToLog("DIST Timed Out: Current Y == " + Double.toString(getYInches()) + "; Goal Y == " + Double.toString(targetY) + "Current X == " + Double.toString(getXInches()) + " Goal X == " + Double.toString(targetX));
				disable();
			}
			
		}
		
	}
	public boolean isOnTarget(){
		return (Math.abs(getYError()) < allowableErrorY) && (Math.abs(getXError()) < allowableErrorX);
	}
	public boolean onTarget(){
		return onTarget;
	}
	/*public void setGoal(double _goalX, ControlWheel _xWheel, double _goalY, ControlWheel _yWheel, double errorX,double errorY, double timeLimit, double maxInput, int checks, int team){
		reset();
		yWheel = _yWheel;
		targetY = _goalY*Constants.FOLLOWER_WHEEL_TICKS_PER_INCH;
		allowableErrorY = errorY*Constants.FOLLOWER_WHEEL_TICKS_PER_INCH;
		targetX = _goalX*Constants.DRIVE_TICKS_PER_INCH;
		allowableErrorX = errorX*Constants.DRIVE_TICKS_PER_INCH;
		timeout = (timeLimit * 1000) + System.currentTimeMillis();
		inputCap = maxInput;
		cyclesToCheck = cyclesOnTarget = checks;
		alliance = team;
		logger.writeToLog("DIST Y Target: " + Double.toString(_goalY) + "\tDIST X Target: " + Double.toString(_goalX));
		enable();
	}*/
	public void setGoal(double _goalX, ControlWheel _xWheel, double _goalY, ControlWheel _yWheel, double errorX,double errorY, double timeLimit, double maxInput, int checks, boolean followNegated){
		reset();
		yWheel = _yWheel;
		xWheel = _xWheel;
		targetY = _goalY;
		allowableErrorY = errorY;
		targetX = _goalX;
		allowableErrorX = errorX;
		timeout = (timeLimit * 1000) + System.currentTimeMillis();
		inputCap = maxInput;
		cyclesToCheck = cyclesOnTarget = checks;
		followerNegated = followNegated;
		logger.writeToLog("DIST Y Target: " + Double.toString(_goalY) + "\tDIST X Target: " + Double.toString(_goalX));
		xPID.reset();
		yPID.reset();
		xPID.setOutputRange(-maxInput, maxInput);
		yPID.setOutputRange(-maxInput, maxInput);
		xPID.setSetpoint(_goalX);
		yPID.setSetpoint(_goalY);
		enable();
	}
	public void setGoal(double _goalX, ControlWheel _xWheel, double _goalY, ControlWheel _yWheel, double error, double timeLimit, double maxInput, int checks, boolean followNegated){
		reset();
		yWheel = _yWheel;
		xWheel = _xWheel;
		targetY = _goalY;
		allowableErrorY = error;
		targetX = _goalX;
		allowableErrorX = error;
		timeout = (timeLimit * 1000) + System.currentTimeMillis();
		inputCap = maxInput;
		cyclesToCheck = cyclesOnTarget = checks;
		followerNegated = followNegated;
		logger.writeToLog("DIST Y Target: " + Double.toString(_goalY) + "\tDIST X Target: " + Double.toString(_goalX));
		xPID.reset();
		yPID.reset();
		xPID.setOutputRange(-maxInput, maxInput);
		yPID.setOutputRange(-maxInput, maxInput);
		xPID.setSetpoint(_goalX);
		yPID.setSetpoint(_goalY);
		enable();
	}
	/*public void setGoal(double _goalX, ControlWheel _xWheel, double _goalY, ControlWheel _yWheel, double error, double timeLimit, double maxInput, int checks, int team){
		reset();
		yWheel = _yWheel;
		targetY = _goalY*Constants.FOLLOWER_WHEEL_TICKS_PER_INCH;
		allowableErrorY = error*Constants.FOLLOWER_WHEEL_TICKS_PER_INCH;
		targetX = _goalX*Constants.DRIVE_TICKS_PER_INCH;
		allowableErrorX = error*Constants.DRIVE_TICKS_PER_INCH;
		timeout = (timeLimit * 1000) + System.currentTimeMillis();
		inputCap = maxInput;
		cyclesToCheck = cyclesOnTarget = checks;
		alliance = team;
		logger.writeToLog("DIST Y Target: " + Double.toString(_goalY) + "\tDIST X Target: " + Double.toString(_goalX));
		enable();
	}*/
	public void reset(){
		cyclesOnTarget = Constants.DIST_CONTROLLER_CYCLE_THRESH;
		onTarget = false;
		updateCurrentPos();
		inputY = 0.0;
		inputX = 0.0;
	}
	private void updateCurrentPos(){		
		if(followerNegated){
			currentPositionY = dt.frontRight.getNegatedFollowerWheelInches();
		}else{
			currentPositionY = dt.frontRight.getFollowerWheelInches();
		}
		switch(xWheel){
			case SWERVE:
				currentPositionX = dt.getX();
				break;
			case FOLLOWER:
				currentPositionX = dt.frontRight.getFollowerWheelInches();
				break;
			default:
				currentPositionX = dt.getX();
				break;
		}
	}
	public void enable(){
		isEnabled = true;
		System.out.println("Distance Controller Enabled");
	}
	public void disable(){
		dt.sendInput(0, 0, 0, 0, false, false, false, false);
		isEnabled = false;
		System.out.println("Distance Controller Disabled");
	}
	private double inputCap(double value){
		if(value > inputCap){
			return inputCap;
		}else if(value < -inputCap){
			return -inputCap;
		}
		if(value > -minCap && value < minCap)
			return 0;
		return value;
	}
	public boolean isEnabled(){
		return isEnabled;
	}
	public double getYInches(){
		return currentPositionY;
	}
	public double getXInches(){
		return currentPositionX;
	}
	private double getXError(){
		return targetX - currentPositionX;
	}
	private double getYError(){
		return targetY - currentPositionY;
	}
	public void setXPID(double p, double i, double d, double f){
		xPID.setPID(p, i, d, f);
	}
	public void setYPID(double p, double i, double d, double f){
		yPID.setPID(p, i, d, f);
	}
	public void negateFollowerWheel(){
		followerNegated = true;
	}
	public void positiveFollowerWheel(){
		followerNegated = false;
	}
}
