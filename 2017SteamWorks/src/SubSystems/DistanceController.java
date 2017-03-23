package SubSystems;

import Helpers.InterpolatingDouble;
import IO.Logger;
import Utilities.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DistanceController {

	private static DistanceController instance;
	private double targetY = 0.0;
	private double targetX = 0.0;
	private double currentPositionY = 0.0;
	private double currentPositionX = 0.0;
	private double allowableError = 1.0;
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
	public boolean useFollower = true;
	private int RED = -1;
	private int BLUE = 1;
	private int alliance = BLUE;
	public DistanceController(){
		dt = Swerve.getInstance();
		logger = Logger.getInstance();
	}
	public static DistanceController getInstance(){
		if(instance == null)
			instance = new DistanceController();
		return instance;
	}
	public void update(){

		SmartDashboard.putBoolean(" Distance Controller Enabled ", isEnabled);
		SmartDashboard.putBoolean(" Dist On Target ", isOnTarget());
		//Util.sdSimpleClosedLoop("Dist X", "Position", currentPositionX/Constants.DRIVE_TICKS_PER_INCH, targetX/Constants.DRIVE_TICKS_PER_INCH);
		//Util.sdSimpleClosedLoop("Dist Y", "Position", currentPositionY/Constants.DRIVE_TICKS_PER_INCH, targetY/Constants.DRIVE_TICKS_PER_INCH);

	
		SmartDashboard.putNumber(" Dist X Set Point ", targetX/Constants.DRIVE_TICKS_PER_INCH);
		if(useFollower){
			SmartDashboard.putNumber(" Dist Y Set Point ", targetY/Constants.FOLLOWER_WHEEL_TICKS_PER_INCH);
			SmartDashboard.putNumber(" Dist Y Position ", currentPositionY/Constants.FOLLOWER_WHEEL_TICKS_PER_INCH);
			SmartDashboard.putNumber(" Dist Y Error ", (targetY - currentPositionY)/Constants.FOLLOWER_WHEEL_TICKS_PER_INCH);
			logger.writeToLog("DIST Y Target: " + Double.toString(targetY/Constants.FOLLOWER_WHEEL_TICKS_PER_INCH) + "\tDIST X Target: " + Double.toString(targetX/Constants.DRIVE_TICKS_PER_INCH));
		}else{
			SmartDashboard.putNumber(" Dist Y Set Point ", targetY/Constants.DRIVE_TICKS_PER_INCH);
			SmartDashboard.putNumber(" Dist Y Position ", currentPositionY/Constants.DRIVE_TICKS_PER_INCH);
			SmartDashboard.putNumber(" Dist Y Error ", (targetY - currentPositionY)/Constants.DRIVE_TICKS_PER_INCH);
			logger.writeToLog("DIST Y Target: " + Double.toString(targetY/Constants.DRIVE_TICKS_PER_INCH) + "\tDIST X Target: " + Double.toString(targetX/Constants.DRIVE_TICKS_PER_INCH));
		}
		SmartDashboard.putNumber(" Dist X Position ", currentPositionX/Constants.DRIVE_TICKS_PER_INCH);
		SmartDashboard.putNumber(" Dist X Error ", (targetX - currentPositionX)/Constants.DRIVE_TICKS_PER_INCH);
		
		
		
	 
		SmartDashboard.putNumber("distInputY", inputY); // this and next line were around line 60 or so,
		SmartDashboard.putNumber("distInputX", inputX); //  after an else block
		if(isEnabled){
			updateCurrentPos();
			if(timeout >= System.currentTimeMillis()){
				if(!isOnTarget()){
					cyclesOnTarget = cyclesToCheck;	
					//double[] errorMap = getError();
					/*
					if(targetY - currentPositionY > 0){
						inputY = getInput(Math.abs(targetY - currentPositionY));
					}
					else{
						inputY = -getInput(Math.abs(targetY - currentPositionY));
					}
					if(targetX - currentPositionX > 0){
						inputX = getInput(Math.abs(targetX - currentPositionX));
					}
					else{
						inputX = -getInput(Math.abs(targetX - currentPositionX));
					}*/
					
					//if(Math.abs(targetY - currentPositionY) > Constants.DIST_CONTROLLER_PID_THRESH_Y * Constants.FOLLOWER_WHEEL_TICKS_PER_INCH){
					if(useFollower){
						if(targetY - currentPositionY > 0){
							inputY = ((targetY - currentPositionY) * Constants.DIST_CONTROLLER_Y_P - (yDistanceTravelled()) * Constants.DIST_CONTROLLER_Y_D) + Constants.DIST_CONTROLLER_F_GAIN;
						}else{
							inputY = ((targetY - currentPositionY) * Constants.DIST_CONTROLLER_Y_P - (yDistanceTravelled()) * Constants.DIST_CONTROLLER_Y_D) - Constants.DIST_CONTROLLER_F_GAIN;
						}
					}else{
						if(targetY - currentPositionY > 0){
							inputY = ((targetY - currentPositionY) * Constants.DIST_CONTROLLER_Y_P_NO_FOLLOWER - (yDistanceTravelled()) * Constants.DIST_CONTROLLER_Y_D_NO_FOLLOWER) + Constants.DIST_CONTROLLER_F_GAIN;
						}else{
							inputY = ((targetY - currentPositionY) * Constants.DIST_CONTROLLER_Y_P_NO_FOLLOWER - (yDistanceTravelled()) * Constants.DIST_CONTROLLER_Y_D_NO_FOLLOWER) - Constants.DIST_CONTROLLER_F_GAIN;
						}
					}
					//}else{
						//inputY = /*((targetY - currentPositionY) * Constants.DIST_CONTROLLER_SMALL_Y_P - (yDistanceTravelled()) * Constants.DIST_CONTROLLER_SMALL_Y_D) + */Constants.DIST_CONTROLLER_F_GAIN;
					//}
					/*if(errorMap[1]>0){
						inputY = getInput(Math.abs(errorMap[1]));
					}else{
						inputY = -getInput(Math.abs(errorMap[1]));
					}
					if(errorMap[0]>0){
						inputX = getInput(Math.abs(errorMap[0]));
					}else{
						inputX = -getInput(Math.abs(errorMap[0]));
					}*/
					
					if(Math.abs(targetX - currentPositionX) > Constants.DIST_CONTROLLER_PID_THRESH_X * Constants.DRIVE_TICKS_PER_INCH){
						inputX = (targetX - currentPositionX) * Constants.DIST_CONTROLLER_X_P - (xDistanceTravelled()) * Constants.DIST_CONTROLLER_X_D;
					}else{
						inputX = (targetX - currentPositionX) * Constants.DIST_CONTROLLER_SMALL_X_P - (xDistanceTravelled()) * Constants.DIST_CONTROLLER_SMALL_X_D;
					}
					
					//logger.writeToLog("DIST: " + Double.toString(currentPositionY/Constants.FOLLOWER_WHEEL_TICKS_PER_INCH) + ": " + Double.toString(currentPositionX/Constants.DRIVE_TICKS_PER_INCH) + " INPUTY: " + Double.toString(inputY) + " INPUTX:" + Double.toString(inputX));

					dt.sendInput(inputCap(inputX),inputCap(inputY) , 0, 0, false, false, false, false); // second false was true
					lastDistanceY = currentPositionY;
					lastDistanceX = currentPositionX;
				}else{
					logger.writeToLog("DIST: " + Double.toString(currentPositionY/Constants.FOLLOWER_WHEEL_TICKS_PER_INCH) + ": " + Double.toString(currentPositionX/Constants.DRIVE_TICKS_PER_INCH));
					dt.sendInput(0, 0, 0, 0, false, false, false, false); // second false was true
					if(cyclesOnTarget <= 0){
						onTarget = true;
						logger.writeToLog("DIST Reached Target: " + Double.toString(timeout-System.currentTimeMillis()) + " ms to spare");
						disable();
					}else{
						cyclesOnTarget--;
					}
				}
			}else{
				dt.sendInput(0, 0, 0, 0, false, false, false, false);
				onTarget = true;
				logger.writeToLog("DIST Timed Out: Current Y == \t" + Double.toString(Math.floor(currentPositionY/Constants.FOLLOWER_WHEEL_TICKS_PER_INCH)) + "; Current X == \t " + Double.toString(currentPositionX/Constants.DRIVE_TICKS_PER_INCH));
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
	public double[] getError(){
		double[] error = new double[2];
		error[0] = (targetX-currentPositionX)/Constants.DRIVE_TICKS_PER_INCH;
		error[1] = (targetY-currentPositionY)/Constants.FOLLOWER_WHEEL_TICKS_PER_INCH;
		return error;
	}
	public void setGoal(double _goalX, double _goalY, double error, double timeLimit, double maxInput, int checks, boolean useFollowerWheel, int team){
		reset();
		if(useFollowerWheel){
			targetY = _goalY*Constants.FOLLOWER_WHEEL_TICKS_PER_INCH;
			allowableError = error*Constants.FOLLOWER_WHEEL_TICKS_PER_INCH;
		}else{
			targetY = _goalY*Constants.DRIVE_TICKS_PER_INCH;
			allowableError = error*Constants.DRIVE_TICKS_PER_INCH;
		}
		targetX = _goalX*Constants.DRIVE_TICKS_PER_INCH;
		//allowableError = error*Constants.DRIVE_TICKS_PER_INCH;
		timeout = (timeLimit * 1000) + System.currentTimeMillis();
		inputCap = maxInput;
		cyclesToCheck = cyclesOnTarget = checks;
		useFollower = useFollowerWheel;
		alliance = team;
		enable();
	}
	public void setGoal(double _goalX, double _goalY, double error, double timeLimit, double maxInput, int checks){
		reset();
		//targetY = _goalY*Constants.DRIVE_TICKS_PER_INCH;// + currentPositionY;//dt.frontLeft.getY();
		targetY = _goalY*Constants.FOLLOWER_WHEEL_TICKS_PER_INCH;
		targetX = _goalX*Constants.DRIVE_TICKS_PER_INCH;// + currentPositionX;//dt.frontLeft.getX();
		//allowableError = error*Constants.DRIVE_TICKS_PER_INCH;
		allowableError = error*Constants.FOLLOWER_WHEEL_TICKS_PER_INCH;
		timeout = (timeLimit * 1000) + System.currentTimeMillis();
		inputCap = maxInput;
		cyclesToCheck = cyclesOnTarget = checks;
		useFollower = true;
		enable();
	}
	public void setOffsetGoal(double _goalX, double _goalY, double error, double timeLimit, double maxInput){
		reset();
		targetY = _goalY*Constants.DRIVE_TICKS_PER_INCH + dt.getRobotY();//frontLeft.getY();// + currentPositionY;//dt.frontLeft.getY();
		targetX = _goalX*Constants.DRIVE_TICKS_PER_INCH + dt.getRobotX();//frontLeft.getX();// + currentPositionX;//dt.frontLeft.getX();
		allowableError = error*Constants.DRIVE_TICKS_PER_INCH;
		timeout = (timeLimit * 1000) + System.currentTimeMillis();
		inputCap = maxInput;
		enable();
	}
	public void reset(){
		cyclesOnTarget = Constants.DIST_CONTROLLER_CYCLE_THRESH;
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
		//currentPositionY = dt.getRobotY();
		if(useFollower){
			if(alliance == BLUE){
				currentPositionY = dt.rearRight.getY();
			}else{
				currentPositionY = -dt.rearRight.getY();
			}
		}else{
			currentPositionY = dt.getRobotY();
		}
		currentPositionX = dt.getRobotX();
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
		dt.sendInput(0, 0, 0, 0, false, false, false, false);
		isEnabled = false;
		//reset();
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
	public double getInput(double error) {
        InterpolatingDouble result;
        result = Constants.kDriveDistanceMap.getInterpolated(new InterpolatingDouble(error));
        if (result != null) {
            return result.value;
        } else {
            return Constants.DIST_MAX_POWER;
        }
    }
}
