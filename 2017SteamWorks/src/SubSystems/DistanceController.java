package SubSystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DistanceController {

	private static DistanceController instance;
	private double targetY = 0.0;
	private double targetX = 0.0;
	private double currentPositionY = 0.0;
	private double currentPositionX = 0.0;
	private double p = 0.030;      //0.0555;
	private double d = 0.000;      //0.12;
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
		SmartDashboard.putNumber("Dist Target", targetY);
		SmartDashboard.putNumber("Dist Current Pos", currentPositionY);
		SmartDashboard.putNumber("Dist Error", targetY - currentPositionY);
		if(isEnabled){
			updateCurrentPos();
			if(timeout >= System.currentTimeMillis()){
				if(!isOnTarget()){
					cyclesOnTarget = onTargetThresh;				
					inputY = (targetY - currentPositionY) * p - (yDistanceTravelled()) * d;
					inputX = (targetX - currentPositionX) * p - (xDistanceTravelled()) * d;
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
		targetY = _goalY + dt.frontLeft.getY();
		targetX = _goalX + dt.frontLeft.getX();
		
		allowableError = error;
		timeout = (timeLimit * 1000) + System.currentTimeMillis();
		inputCap = maxInput;
		enable();
	}
	public void reset(){
		cyclesOnTarget = onTargetThresh;
		onTarget = false;
		currentPositionY = 0.0;
		inputY = 0.0;
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
		reset();
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
