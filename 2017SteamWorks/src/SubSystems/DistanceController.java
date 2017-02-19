package SubSystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DistanceController {

	private static DistanceController instance;
	private double target = 0.0;
	private double currentPosition = 0.0;
	private double p = 0.054;
	private double d = 0.12;
	private double allowableError = 1.0;
	private boolean onTarget = false;
	private int onTargetThresh = 30;
	private int cyclesOnTarget = 0;
	private Swerve dt;
	private double input = 0.0;
	private double lastDistance = 0.0;
	private boolean isEnabled = false;
	private double inputCap = 0.7;
	
	public enum Direction{
		X,Y,BOTH
	}
	private Direction axis = Direction.Y;
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
		SmartDashboard.putNumber("Dist Target", target);
		SmartDashboard.putNumber("Dist Current Pos", currentPosition);
		if(isEnabled){
			updateCurrentPos();
			if(getStatus()){
				cyclesOnTarget = onTargetThresh;				
				input = (target - currentPosition) * p - (distanceTravelled()) * d;
				switch(axis){
				case Y:
					dt.sendInput(0, inputCap(input), 0, 0, false, true, false);
					break;
				case X:
					dt.sendInput(inputCap(input),0 , 0, 0, false, true, false);
					break;
				case BOTH:
					
					break;
				}				
				lastDistance = currentPosition;
			}else{
				dt.sendInput(0, 0, 0, 0, false, true, false);
				if(cyclesOnTarget <= 0){
					onTarget = true;
					disable();
				}else{
					cyclesOnTarget--;
				}
			}
		}
	}
	public boolean getStatus(){
		return Math.abs(currentPosition - target) > allowableError;
	}
	public boolean onTarget(){
		return onTarget;
	}
	public void setGoal(double _goal, Direction _d, double error, double maxInput){
		reset();
		switch(axis){
		case Y:
			target = _goal + dt.frontLeft.getY();
			break;
		case X:
			target = _goal + dt.frontLeft.getX();
			break;
		case BOTH:
			
			break;
		}
		allowableError = error;
		axis = _d;
		inputCap = maxInput;
		enable();
	}
	public void reset(){
		cyclesOnTarget = onTargetThresh;
		onTarget = false;
		currentPosition = 0.0;
		input = 0.0;
	}
	private void updateCurrentPos(){
		switch(axis){
		case Y:
			currentPosition = dt.frontLeft.getY();
			break;
		case X:
			currentPosition = dt.frontLeft.getX();
			break;
		case BOTH:
			
			break;
		}		 
	}
	private double distanceTravelled(){
		return currentPosition - lastDistance;
	}
	public void enable(){
		isEnabled = true;
	}
	public void disable(){
		isEnabled = false;
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
	public void setDirection(Direction _d){
		axis = _d;
	}
}
