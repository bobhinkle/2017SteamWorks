package SubSystems;

import Utilities.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DistanceController {

	private static DistanceController instance;
	private double targetY = 0.0;
	private double targetX = 0.0;
	private double currentPositionY = 0.0;
	private double currentPositionX = 0.0;
	private double p = 0.000025;      //0.0555;
	private double d = 0.00001;      //0.12;
	private double smallP = 0.00001;
	private double smallD = 0.0;
	private double allowableError = 1.0;
	private boolean onTarget = false;
	private int onTargetThresh = 30;
	private int cyclesOnTarget = 0;
	private Swerve dt;
	private double inputY = 0.0;
	private double inputX = 0.0;
	private double lastDistanceY = 0.0;
	private double lastDistanceX = 0.0;
	private boolean isEnabled = false;
	private double timeout = 0;
	private double inputCap = 0.7;
	private double inputCap2 = 0.3;
	private double smallPIDThresh = 5.0;
	public enum Direction{
		X,Y,BOTH
	}
	public DistanceController(){
		dt = Swerve.getInstance();
	}
	public static DistanceController getInstance(){
		if(instance == null)
			instance = new DistanceController();
		return instance;
	}
	public void update(){
		SmartDashboard.putBoolean("Dist_Enabled", isEnabled);
		SmartDashboard.putNumber("Dist Target X", targetX);
		SmartDashboard.putNumber("Dist Target Y", targetY);
		SmartDashboard.putNumber("Dist Target X (in)", targetX/Constants.DRIVE_TICKS_PER_INCH);
		SmartDashboard.putNumber("Dist Target Y (in)", targetY/Constants.DRIVE_TICKS_PER_INCH);
		SmartDashboard.putNumber("Dist Pos X", currentPositionX/Constants.DRIVE_TICKS_PER_INCH);
		SmartDashboard.putNumber("Dist Pos Y", currentPositionY/Constants.DRIVE_TICKS_PER_INCH);
		SmartDashboard.putNumber("Dist Error X", targetX - currentPositionX);
		SmartDashboard.putNumber("Dist Error Y", targetY - currentPositionY);
		SmartDashboard.putBoolean("Dist On Target", isOnTarget());
		if(isEnabled){
			updateCurrentPos();
			if(timeout >= System.currentTimeMillis()){
				if(!isOnTarget()){
					cyclesOnTarget = onTargetThresh;	
					if(Math.abs(targetY - currentPositionY) > smallPIDThresh * Constants.DRIVE_TICKS_PER_INCH){
						inputY = (targetY - currentPositionY) * p - (yDistanceTravelled()) * d;
					}else{
						inputY = (targetY - currentPositionY) * smallP - (yDistanceTravelled()) * smallD;
					}
					if(Math.abs(targetX - currentPositionX) > smallPIDThresh * Constants.DRIVE_TICKS_PER_INCH){
						inputX = (targetX - currentPositionX) * p - (xDistanceTravelled()) * d;
					}else{
						inputX = (targetX - currentPositionX) * smallP - (xDistanceTravelled()) * smallD;
					}
					
					SmartDashboard.putNumber("distInputY", inputY);
					SmartDashboard.putNumber("distInputX", inputX);
					dt.sendInput(inputCap(inputX),inputCap(inputY) , 0, 0, false, true, false);
					lastDistanceY = currentPositionY;
					lastDistanceX = currentPositionX;
				}else{
					dt.sendInput(0, 0, 0, 0, false, true, false);
					if(cyclesOnTarget <= 0){
						onTarget = true;
						disable();
					}else{
						cyclesOnTarget--;
					}
				}
			}else{
				dt.sendInput(0, 0, 0, 0, false, true, false);
				onTarget = true;
				disable();
			}
		}
	}
	public boolean isOnTarget(){
		return (Math.abs(currentPositionY - targetY) < allowableError) && (Math.abs(currentPositionX - targetX) < allowableError);
	}
	public boolean onTarget(){
		return onTarget;
	}
/**	public void linearMotion(double goal, DistanceController.Direction direction, double error, double inputCap){
    	int timeout = 0;
    	dist.setGoal(goal, direction, error, inputCap);
		while(!dist.onTarget() && (timeout < 300)){
			Timer.delay(0.01);
			timeout++;
		}
    }/**/
	public void setGoal(double _goalX, double _goalY, double error, double timeLimit, double maxInput){
		reset();
		targetY = _goalY*Constants.DRIVE_TICKS_PER_INCH;// + currentPositionY;//dt.frontLeft.getY();
		targetX = _goalX*Constants.DRIVE_TICKS_PER_INCH;// + currentPositionX;//dt.frontLeft.getX();
		//targetY = (int) targetY;	// cast these doubles as ints because they're now in ticks or whatever
		//targetX = (int) targetX;
		allowableError = error*Constants.DRIVE_TICKS_PER_INCH;
		timeout = (timeLimit * 1000) + System.currentTimeMillis();
		inputCap = maxInput;
		enable();
	}
	public void setOffsetGoal(double _goalX, double _goalY, double error, double timeLimit, double maxInput){
		reset();
		targetY = _goalY*Constants.DRIVE_TICKS_PER_INCH + dt.frontLeft.getY();// + currentPositionY;//dt.frontLeft.getY();
		targetX = _goalX*Constants.DRIVE_TICKS_PER_INCH + dt.frontLeft.getX();// + currentPositionX;//dt.frontLeft.getX();
		//targetY = (int) targetY;	// cast these doubles as ints because they're now in ticks or whatever
		//targetX = (int) targetX;
		allowableError = error*Constants.DRIVE_TICKS_PER_INCH;
		timeout = (timeLimit * 1000) + System.currentTimeMillis();
		inputCap = maxInput;
		enable();
	}
	public void reset(){
		cyclesOnTarget = onTargetThresh;
		onTarget = false;
		/**/
		updateCurrentPos();
		/*/
		currentPositionY = 0.0;
		/**/
		inputY = 0.0;
		inputX = 0.0;
	}
	private void updateCurrentPos(){
		currentPositionY = dt.frontLeft.getY();
		currentPositionX = dt.frontLeft.getX();
	}
	private double yDistanceTravelled(){
		return currentPositionY - lastDistanceY;
	}
	private double xDistanceTravelled(){
		return currentPositionX - lastDistanceX;
	}
	public void enable(){
		isEnabled = true;
	}
	public void disable(){
		isEnabled = false;
		//reset();
	}
	private double inputCap(double value){
		if(value > inputCap){
			return inputCap;
		}else if(value < -inputCap){
			return -inputCap;
		}
		return value;
	}
	public boolean isEnabled(){
		return isEnabled;
	}
}
