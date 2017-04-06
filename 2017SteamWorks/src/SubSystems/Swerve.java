package SubSystems;

import java.util.Set;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import com.sun.glass.ui.Robot;

import Helpers.InterpolatingDouble;
import Utilities.Constants;
import Utilities.Ports;
import Utilities.Util;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Swerve{
	private static Swerve instance = null;
	private double xInput;
	private double yInput;
	private double rotateInput;
	
	double rotationOnTarget = 0;

	double _targetAngle = 0.0;
	double rotationCorrection;
	int _printLoops = 0;
	private Intake intake;
	private int onTarget = Constants.MIN_CYCLES_HEADING_ON_TARGET;

	private double lastYPos = 0.0;
	private double lastXPos = 0.0;
	private double lastYVel = 0.0;
	private double lastXVel = 0.0;
	private double AccelX = 0.0;
	private double AccelY = 0.0;
	private int cyclesSinceLastUpdate = 0;
	
	private SwerveDriveModule frontLeft;
	public SwerveDriveModule frontRight;
	private SwerveDriveModule rearLeft;
	public SwerveDriveModule rearRight;
	private boolean disableUpdates = false;
	private boolean enableRotation = false;
	private boolean isHanging = false;
	public void startHanging(){
		isHanging = true;
	}
	public void stopHanging(){
		isHanging = false;
	}
	public void enableRotation(){
		enableRotation = true;
	}
	public void disableRotation(){
		enableRotation = false;
	}
	private final double RED = -1;
	private final double BLUE = 1;
	
	public enum AnglePresets{
		ZERO,NINETY,ONE_EIGHTY, TWO_SEVENTY
	}
	
	public enum DriveControlState {
        OPEN_LOOP, BASE_LOCKED, VELOCITY_SETPOINT, VELOCITY_HEADING_CONTROL, PATH_FOLLOWING_CONTROL
    }
    
    private boolean isBrakeMode_ = false;
    
    protected static final int kVelocityControlSlot = 0;
    protected static final int kBaseLockControlSlot = 1;
	/*
	//private double x = 0.0;
	//private double y = 0.0;
	//private double xOffset = 8.0;
	//private double yOffset = 11.0;
	/**/
	//Pigeon IMU Data
	
	/** 
	 * Sets the robot's target heading.
	 * <p>
	 * In an autonomous subroutine:
	 *  <ul>
	 *   <li>Call this function before calling the next translation with {@link SubSystems.DistanceController#setGoal(double, double, double, double, double) dist.setGoal(...)}.</li>
	 *   <li>Set <code>rotation = true</code>, even though we don't want it to rotate in place. It'll just rotate during the translation.</li>
	 *  </ul>
	 * </p>
	 * @param goal the heading which we wish the robot to assume
	 * @param rotation whether or not the given heading should be approached with an in-place rotation
	 *  */
	public void setHeading(double goal,boolean rotation){		
		if(rotation){
			headingController = HeadingController.Rotation;
			rotationOnTarget = Constants.MIN_CYCLES_HEADING_ON_TARGET;
		}
		_targetAngle = Util.continousAngle(goal,intake.getCurrentAngle());
	}

	/**
	 * States of the {@link Swerve} {@link Swerve#headingController heading controller}.
	 * */
	enum HeadingController{
		/** 
		 * Set the {@link Swerve#headingController heading controller} to <code>Off</code> when we're
		 *  adjusting the robot's heading manually, the robot is performing certain maneuvers,
		 *  or the {@link Intake#pidgeyGood() Pigeon} isn't working properly.
		 */
		Off,
		/**
		 * Set the {@link Swerve#headingController heading controller} to <code>Heading</code> when we
		 *  want the robot to maintain its current heading while driving.
		 */
		Heading, Rotation, Reset 
	}
	public HeadingController headingController = HeadingController.Off;
	
	public Swerve(){
		intake = Intake.getInstance();		
		frontLeft  = new SwerveDriveModule(Ports.FRONT_LEFT_ROTATION,Ports.FRONT_LEFT_DRIVE,2,Constants.FRONT_LEFT_TURN_OFFSET);
		frontRight = new SwerveDriveModule(Ports.FRONT_RIGHT_ROTATION,Ports.FRONT_RIGHT_DRIVE,1,Constants.FRONT_RIGHT_TURN_OFFSET);
		rearLeft   = new SwerveDriveModule(Ports.REAR_LEFT_ROTATION,Ports.REAR_LEFT_DRIVE,3,Constants.REAR_LEFT_TURN_OFFSET);
		rearRight  = new SwerveDriveModule(Ports.REAR_RIGHT_ROTATION,Ports.REAR_RIGHT_DRIVE,4,Constants.REAR_RIGHT_TURN_OFFSET);
	}
	public static Swerve getInstance()
    {
        if( instance == null )
            instance = new Swerve();
        return instance;
    }
	
	public void swerveTrack(){
//		double adjust = intake.getCurrentAngle() - Vision.getAngle();
//    	setHeading(adjust,false);
	}
	
	
	 /* cap corrective turning throttle to 30 percent of forward throttle */
	/** Heading of the robot at the beginning of the drivetrain's {@link Swerve#update() update} cycle */
	private double currentRobotHeading;// = intake.getCurrentAngle();
	private void refreshRobotHeading() {currentRobotHeading = Math.toRadians(intake.getCurrentAngle());}
	private double robotX;
	private double robotY;

	private double x_offset = 0.0;
	private double y_offset = 0.0;
//	private void findRobotX() {robotX = rearLeft.getX() + (Constants.RADIUS_CENTER_TO_MODULE * Math.cos(currentRobotHeading+Constants.ANGLE_FRONT_MODULE_CENTER));} // both should be 90-(currentRobotHeading+C...Angle_Front...) with a + at the front
//	private void findRobotY() {robotY = rearLeft.getY() - (Constants.RADIUS_CENTER_TO_MODULE * Math.sin(currentRobotHeading+Constants.ANGLE_FRONT_MODULE_CENTER));}
	 // both should be 90-(currentRobotHeading+C...Angle_Front...) with a + at the front

	
//	private void findRobotX() {robotX = rearLeft.getX() + (Constants.RADIUS_CENTER_TO_MODULE * Math.cos(Math.toRadians(90)-(currentRobotHeading+Constants.ANGLE_FRONT_MODULE_CENTER)));} // both should be 90-(currentRobotHeading+C...Angle_Front...) with a + at the front
//	private void findRobotY() {robotY = rearLeft.getY() + (Constants.RADIUS_CENTER_TO_MODULE * Math.sin(Math.toRadians(90)-(currentRobotHeading+Constants.ANGLE_FRONT_MODULE_CENTER)));}
	
	private void findRobotX() {robotX = rearRight.getX() - (Constants.RADIUS_CENTER_TO_MODULE * Math.cos(Math.atan(Constants.WHEELBASE_LENGTH/Constants.WHEELBASE_WIDTH)-currentRobotHeading));} // both should be 90-(currentRobotHeading+C...Angle_Front...) with a + at the front
	private void findRobotY() {robotY = rearRight.getY() + (Constants.RADIUS_CENTER_TO_MODULE * Math.sin(Math.atan(Constants.WHEELBASE_LENGTH/Constants.WHEELBASE_WIDTH)-currentRobotHeading));}
	
	private void findRobotCenter() {findRobotX(); findRobotY();}
	public double getRobotX() {return robotX;}
	public double getRobotY() {return robotY;}
	public double getRobotXInch() {return robotX/Constants.DRIVE_TICKS_PER_INCH;}
	public double getRobotYInch() {return robotY/Constants.DRIVE_TICKS_PER_INCH;}
	public double getX(){return getRobotXInch();}
	public double getY(){return getRobotYInch();}
	public double getXWithOffsets(){return getRobotXInch()+x_offset;}
	public double getYWithOffsets(){return getRobotYInch()+y_offset;}
	public void setOffsets(double _x, double _y){
		x_offset = _x - getX(); y_offset = _y - getY(); 
	}
	/**/
	
	public void sendInput(double x, double y, double rotateX,double rotateY,boolean halfPower,boolean robotCentric,boolean moonManeuver,boolean lowPower){

/*		SmartDashboard.putNumber("X StickL", x);
		SmartDashboard.putNumber("Y StickL", y);
		SmartDashboard.putNumber("X Stick", rotateX);
		SmartDashboard.putNumber("Y Stick", rotateY);*/
		if(moonManeuver){
			headingController = HeadingController.Off;
			yInput = 0.0;
			xInput = -(rotateX * 0.2) * ((Math.abs(y)*2)+1) ;
			rotateInput = rotateX * 0.2;
//			SmartDashboard.putNumber("xMoon", xInput);
//			SmartDashboard.putNumber("rMoon", rotateInput);
		}
		else{
			if(!intake.pidgeyGood()){
				headingController = HeadingController.Off;
			}
			if(Math.abs(rotateX) >= Constants.STICK_DEAD_BAND){
				headingController = HeadingController.Off;
					rotateInput = rotateX;
					
			}else if(Math.abs(rotateX) < Constants.STICK_DEAD_BAND && headingController == HeadingController.Off){
				headingController = HeadingController.Heading;
				_targetAngle = intake.getCurrentAngle();
				rotateInput = 0.0;
			}else{
				rotateInput = 0.0;
			}
			double angle = intake.getCurrentAngle()/180.0*Math.PI; // radians
			double angleDiff = Math.abs(getError());
			if(enableRotation){
				headingController = HeadingController.Rotation;
			}
			switch(headingController){
			case Off:
				rotationCorrection = 0.0;
				rotateInput *= 0.5;
				SmartDashboard.putString(" Heading Controller Mode ", "OFF");
				break;
			case Heading:
				if(angleDiff >=5){
					rotationCorrection = Util.calcPID(Constants.SWERVE_HEADING_BIG_P, 0.0, Constants.SWERVE_HEADING_BIG_D, 0.0, getError(), intake.currentAngularRate, Constants.SWERVE_HEADING_BIG_MAX_CORRECTION_HEADING);
				}else{
					if(angleDiff > 0.5){	
						rotationCorrection = Util.calcPID(Constants.SWERVE_HEADING_GAIN_P, 0.0, Constants.SWERVE_HEADING_GAIN_D, 0.0, getError(), intake.currentAngularRate, Constants.SWERVE_HEADING_MAX_CORRECTION_HEADING);
					}else{
						rotationCorrection = 0.0;
					}
				}
				SmartDashboard.putString(" Heading Controller Mode ", "Heading");
				break;
			case Rotation:	
				
				rotationCorrection = Util.calcPID(Constants.SWERVE_TURNING_GAIN_P, 0.0, Constants.SWERVE_TURNING_GAIN_D, Constants.SWERVE_TURNING_GAIN_FF, getError(), intake.currentAngularRate, Constants.SWERVE_ROTATION_INPUT_CAP);
				
				if(angleDiff < Constants.SWERVE_ROTATION_HEADING_ON_TARGET_THRESHOLD){
					rotationOnTarget--;
					rotationCorrection = 0.0;
					if(rotationOnTarget <= 0){ // `rotationOnTarget' was `onTarget'
						headingController = HeadingController.Heading;
					}
				}else{
					rotationOnTarget = Constants.MIN_CYCLES_HEADING_ON_TARGET;
				}
				SmartDashboard.putString(" Heading Controller Mode ", "rotation");
				break;
			case Reset:
				SmartDashboard.putString(" Heading Controller Mode ", "reset");
				break;
			default:
				
				break;
			}
			if(Math.abs(x) < Constants.STICK_DEAD_BAND){
				x = 0;
			}
			if(Math.abs(y) < Constants.STICK_DEAD_BAND){
				y = 0;
			}
			if(y ==0 && x == 0 && headingController == HeadingController.Heading){
				headingController = HeadingController.Off;
			}
			if((y != 0 || x != 0) && headingController == HeadingController.Rotation ){
				headingController = HeadingController.Heading;
			}
			SmartDashboard.putNumber("ROTATE_CORRECT", rotationCorrection);
			if(halfPower || isHanging){
				y = y * 0.45;
				x = x * 0.45;	
				rotateInput *= 0.75;
			}else if(lowPower){
				y = y * 0.3;
				x = x * 0.3;
			}else{
				y = y * 1.0;
				x = x * 1.0;
			}
			rotateInput = rotateInput + rotationCorrection;
			
			if(robotCentric){
				xInput = x;
				yInput = y;
			}
			else{
				double tmp = (y* Math.cos(angle)) + (x * Math.sin(angle));
				xInput = (-y * Math.sin(angle)) + (x * Math.cos(angle));
				yInput = tmp;			
			}
		}
		SmartDashboard.putNumber("Rotation Input", rotateInput);
		SmartDashboard.putNumber(" Heading Set Point ", _targetAngle); // moved to Swerve.update()
	}
	
	
	/**
	 * The {@link SwerveDriveModule} class controls the actions and properties of individual Swerve modules.
	 * <p>
	 *  A module is composed of a {@link SwerveDriveModule#driveMotor drive motor} and a {@link SwerveDriveModule#rotationMotor rotation motor}.
	 *  The module uses the rotation motor's position to calculate the angle of the drive module, relative to the robot's forward
	 * */
	public class SwerveDriveModule{
		private CANTalon rotationMotor;
		public CANTalon driveMotor;
		private int moduleID;
		private int absolutePosition;
		private double x = 0.0;
		private double y = 0.0;
		private double offSet = 0.0;
		private boolean reversePower = false;
		private double lastEncPosition = 0.0;
		
		private double currentDriveEncoderPosition;
		private double getCurrentDriveEncoderPosition() {return currentDriveEncoderPosition;}
		private void setCurrentDriveEncoderPosition(double _currentEncPosition) {currentDriveEncoderPosition = _currentEncPosition;}
		
		private double currentIntakeAngle;
		private double getCurrentIntakeAngle() {return currentIntakeAngle;}
		private void setCurrentIntakeAngle(double _currentIntakeAngle) {currentIntakeAngle = _currentIntakeAngle;}
		
		private double currentModuleAngle;
		private double getCurrentModuleAngle() {return currentModuleAngle;}
		private void setCurrentModuleAngle(double _currentModuleAngle) {currentModuleAngle = _currentModuleAngle;}
		
		private double totalDistanceTravelled = 0.0;
		public double getTotalDistanceTravelled() {return totalDistanceTravelled;}
		public void setTotalDistanceTravelled(double dist) {totalDistanceTravelled = dist;}
		
		private int rotationOffsetClicks = 0;
		public int getRotationOffsetClicks() {return rotationOffsetClicks;}
		public void setRotationOffsetClicks(int offset) {rotationOffsetClicks = offset;}

		private int robotRotationOffsetClicks = 0;
		public int getRobotRotationOffsetClicks() {return robotRotationOffsetClicks;}
		public void setRobotRotationOffsetClicks(int offset) {robotRotationOffsetClicks = offset;}
		
		private boolean isRotating = false;
		public boolean isRotating() {return isRotating;}
		private boolean isTravelling = false;
		public boolean isTravelling() {return isTravelling;}
		
		private double relativeTickZero = 0;
		private double relativeTickCount(){
			return getCurrentDriveEncoderPosition() - relativeTickZero;
		}
		
		public void updateCoord(){		
			if(moduleID != Constants.FOLLOWER_WHEEL_MODULE_ID){
				setCurrentDriveEncoderPosition(driveMotor.getEncPosition());
				setCurrentIntakeAngle(intake.getCurrentAngle());
				setCurrentModuleAngle(rotationMotor.get()-offSet);
	//			SmartDashboard.putString("updateCoord():setCurrentModule"+Integer.toString(moduleID)+"Angle", Double.toString(Util.boundAngle0to360Degrees(getCurrentModuleAngle()))+" / "+Double.toString(Util.boundAngle0to360Degrees(rotationMotor.get())));
				double distanceTravelled = -(getCurrentDriveEncoderPosition()-lastEncPosition);// * Constants.DRIVE_INCHES_PER_CLICK; //0.00180143;*Constants.DRIVE_CLICKS_PER_INCH; // inches
		        totalDistanceTravelled -= distanceTravelled; // inches
	//	        SmartDashboard.putNumber("distanceTravelled"+Integer.toString(moduleID), distanceTravelled);
	//	        SmartDashboard.putBoolean(Integer.toString(moduleID) + " Travelling ", isTravelling());
	//	        SmartDashboard.putNumber(Integer.toString(moduleID) + " Distance (in) ", totalDistanceTravelled);
	//	        SmartDashboard.putNumber(Integer.toString(moduleID) + " cos = ", Math.cos(Math.toRadians(360-rotationMotor.get()+90)));
	//	        SmartDashboard.putNumber(Integer.toString(moduleID) + " sin = ", Math.sin(Math.toRadians(360-rotationMotor.get()+90)));
		        //should be negative for the comp bot, positive for the pbot
		        double dx = -distanceTravelled * Math.cos(Math.toRadians(getCurrentIntakeAngle()+getCurrentModuleAngle()+90));	// should be ``double dx = distanceTravelled * Math.cos(Math.toRadians(90-(getCurrentIntakeAngle()+getCurrentModuleAngle())));''
		        //should be positive for comp bot, negative for the pbot
		        double dy = distanceTravelled * Math.sin(Math.toRadians(getCurrentIntakeAngle()+getCurrentModuleAngle()+90));	// should be ``double dx = distanceTravelled * Math.sin(Math.toRadians(90-(getCurrentIntakeAngle()+getCurrentModuleAngle())));''
		        x += dx;
		        y += dy;
	//	        SmartDashboard.putNumber("DRV_X" + Integer.toString(moduleID), x);
	//	        SmartDashboard.putNumber("DRV_Y" + Integer.toString(moduleID), y);
	//	        SmartDashboard.putNumber(Integer.toString(moduleID) + " dX ", dx*1000);
	//	        SmartDashboard.putNumber(Integer.toString(moduleID) + " dY ", dy*1000);
		        if(moduleID == Constants.SWERVE_ENCODER_MODULE_ID){
			        SmartDashboard.putNumber(Integer.toString(moduleID) + " X (tick) ", x);
			        SmartDashboard.putNumber(Integer.toString(moduleID) + " Y (tick) ", y);
		 	        SmartDashboard.putNumber(Integer.toString(moduleID) + " X (in) ", x/Constants.DRIVE_TICKS_PER_INCH);
			        SmartDashboard.putNumber(Integer.toString(moduleID) + " Y (in) ", y/Constants.DRIVE_TICKS_PER_INCH);
		        }
		        //SmartDashboard.putNumber(Integer.toString(moduleID) + "VOL", driveMotor.getOutputVoltage());
				//SmartDashboard.putNumber(Integer.toString(moduleID) + "ROT_VOL", rotationMotor.getOutputVoltage());
				SmartDashboard.putBoolean(Integer.toString(moduleID) + "reversePower", reversePower);
				lastEncPosition = getCurrentDriveEncoderPosition();			
			}else{
				setCurrentDriveEncoderPosition(driveMotor.getEncPosition());
				setCurrentIntakeAngle(intake.getCurrentAngle());
				setCurrentModuleAngle(rotationMotor.get()-offSet);
				y = driveMotor.getEncPosition();
				SmartDashboard.putNumber(Integer.toString(moduleID) + " Y (tick) ", y);
				SmartDashboard.putNumber(Integer.toString(moduleID) + " Y (in) ", getFollowerWheelInches());
				SmartDashboard.putNumber(Integer.toString(moduleID) + " Y (in) Graph", getFollowerWheelInches());
				//SmartDashboard.putNumber(Integer.toString(moduleID) + "VOL", driveMotor.getOutputVoltage());
				//SmartDashboard.putNumber(Integer.toString(moduleID) + "ROT_VOL", rotationMotor.getOutputVoltage());
				SmartDashboard.putBoolean(Integer.toString(moduleID) + "reversePower", reversePower);
			}
		}
		public double getX(){return x;}
		public double getY(){return y;}
		public double getBlueY(){return y*BLUE;}
		public double getRedY(){return y*RED;}
		public double getXInch(){return x/Constants.DRIVE_TICKS_PER_INCH;}
		public double getYInch(){return y/Constants.DRIVE_TICKS_PER_INCH;}
		public double getNegatedFollowerWheelInches(){return -(y/Constants.FOLLOWER_WHEEL_TICKS_PER_INCH);}
		public double getFollowerWheelInches(){return (y/Constants.FOLLOWER_WHEEL_TICKS_PER_INCH);}
		public void resetCoord(AnglePresets i){
			x = 0;
			y = 0;
			driveMotor.setEncPosition(0);
			setRotationOffsetClicks(0);
			setTotalDistanceTravelled(0);
			lastEncPosition = 0;
			setCurrentDriveEncoderPosition(0);
//			relativeTickZero = getCurrentDriveEncoderPosition();
			initModule(i);
		}
		public void resetFollower(){
			if(moduleID == Constants.FOLLOWER_WHEEL_MODULE_ID){
				driveMotor.setEncPosition(0);
			}
		}
		public void debugValues(){
			if(wheelError() >= Constants.TURNING_DETECT_THRESHOLD) {isRotating = true;} else {isRotating = false;}
			if(isRotating || Math.abs(getCurrentDriveEncoderPosition()-lastEncPosition) <= Constants.DRIVING_DETECT_THRESHOLD /*clicks*/) {isTravelling = false;} else {isTravelling = true;}
			//Note #3
			//SmartDashboard.putNumber(Integer.toString(moduleID) + " Current", driveMotor.getOutputCurrent());
			SmartDashboard.putNumber(Integer.toString(moduleID) + " Rotation Angle (deg) ", Util.boundAngle0to360Degrees(getCurrentModuleAngle()));
//			SmartDashboard.putNumber("DRV_" + Integer.toString(moduleID), driveMotor.get());
//			SmartDashboard.putNumber(Integer.toString(moduleID) + " Bearing ", Util.boundAngle0to360Degrees(getCurrentAngle()-offSet)); // `-offSet' was commented out
//			SmartDashboard.putNumber("GOAL " + Integer.toString(moduleID), Util.boundAngle0to360Degrees(rotationMotor.getSetpoint()-(360-offSet)));
//			SmartDashboard.putNumber("W_ERR" + Integer.toString(moduleID), Util.boundAngle0to360Degrees(wheelError()));
//			SmartDashboard.putNumber(Integer.toString(moduleID)+ " Raw Encoder Clicks ", relativeTickCount()/*getCurrentEncPosition()*/);
//			SmartDashboard.putBoolean(Integer.toString(moduleID) + " Rotating ", isRotating());
//			SmartDashboard.putNumber(Integer.toString(moduleID) + " Rotation Clicks ", getRotationOffsetClicks());
//			SmartDashboard.putNumber(Integer.toString(moduleID) + " Driven Clicks ", getCurrentEncPosition() - getRotationOffsetClicks());
//			SmartDashboard.putNumber(Integer.toString(moduleID) + "Inches since last zero: ", getCurrentEncPosition()/Constants.DRIVE_TICKS_PER_INCH);//Constants.DRIVE_INCHES_PER_CLICK);
		}
		public SwerveDriveModule(int rotationMotorPort, int driveMotorPort,int moduleNum,double _offSet){
			rotationMotor = new CANTalon(rotationMotorPort);
			rotationMotor.setPID(2, 0.0, 30, 0.0, 0, 0.0, 0);
			driveMotor = new CANTalon(driveMotorPort);	    	
//	    	driveMotor.changeControlMode(TalonControlMode.);
//	    	driveMotor.set(driveMotor.getPosition());
			loadProperties();
			moduleID = moduleNum;        
			offSet = _offSet;
			double targetAngle = 0;
			driveMotor.reverseOutput(false);
			
		}
		public void disableReverse(){
			driveMotor.configPeakOutputVoltage(+12f, -0f);
		}
		public void enableReverse(){
			driveMotor.configPeakOutputVoltage(+12f, -12f);
		}
		public void enableBreakMode(){
			driveMotor.enableBrakeMode(true);
		}
		public void disableBreakMode(){
			driveMotor.enableBrakeMode(false);
		}
		public void setGoal(double goalAngle)
	    {			
			double newAngle = Util.continousAngle(goalAngle-(360-offSet),getCurrentAngle());
			if(Util.shortestPath(getCurrentAngle(),goalAngle-(360-offSet)) || cannotReverse){
				rotationMotor.set(newAngle);
				reversePower = false;				
			}else{
				reversePower = true;
				rotationMotor.set(Util.continousAngle(goalAngle + 180.0-(360-offSet),getCurrentAngle()));
			}	     
			
	    }
		public double wheelError(){
			return Math.abs(rotationMotor.getSetpoint() - getCurrentAngle());
		}
	    public final void loadProperties()
	    {
	    	absolutePosition = rotationMotor.getPulseWidthPosition() & 0xFFF;
	    	rotationMotor.setEncPosition(absolutePosition);
	    	rotationMotor.setFeedbackDevice(FeedbackDevice.AnalogEncoder);
	    	rotationMotor.reverseSensor(false);
	    	rotationMotor.reverseOutput(true);
	    	rotationMotor.configPotentiometerTurns(360);
	    	rotationMotor.configNominalOutputVoltage(+0f, -0f);
	    	rotationMotor.configPeakOutputVoltage(+7f, -7f);
	    	rotationMotor.setAllowableClosedLoopErr(0); 
	    	rotationMotor.changeControlMode(TalonControlMode.Position);
	    	rotationMotor.setPID(5.0, 0.0, 40.0, 0.00, 0, 0.0, 0);
	    	rotationMotor.setProfile(0);
	    	rotationMotor.set(rotationMotor.getEncPosition());
	    	driveMotor.setEncPosition(0);
	    	driveMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
	    	driveMotor.configEncoderCodesPerRev(360);
	    	driveMotor.configNominalOutputVoltage(+0f, -0f);
	    	driveMotor.configPeakOutputVoltage(+12f, -0f);
	    	driveMotor.setAllowableClosedLoopErr(0);
	    	reversePower = false;
	    }
	    
	    public void setTeleRampRate(){
	    	driveMotor.setVoltageRampRate(48.0);
	    }
	    public void setAutoRamprate(){
	    	driveMotor.setVoltageRampRate(0.0);
	    }
	    
	   // 2017-03-05 Added because of a potential logic error
    	/** Determines whether we're close enough to a wheel's desired angle to start driving that wheel.*/
	    public boolean isAngleOkay(){if(wheelError() < Constants.TURNING_ADD_POWER_THRESHOLD) {return true;} else {return false;}}
    	/** Determines whether we're close enough to a wheel's desired angle to start driving that wheel, then actually applies power to the wheel. */
	    public void setDriveSpeed(double power){
	    	if(wheelError() < Constants.TURNING_ADD_POWER_THRESHOLD) {
	    		if(reversePower)
	    			driveMotor.set(power);
	    		else
	    			driveMotor.set(-power);}
//	    	System.out.println("Drive motor power: " + Double.toString(power));
	   	}
	    public void stopDriveMotor(){
	    	enableBreakMode();
	    	driveMotor.set(0);
	    }
	    public void stopRotation(){
	    	rotationMotor.set(rotationMotor.get());
	    }
		public double getCurrentAngle(){
			return rotationMotor.get();
		}
		public void initModule(AnglePresets i){
			switch(i){
			case ZERO: //0
				/**/
				x = Constants.RADIUS_CENTER_TO_MODULE * Math.cos(Math.atan(Constants.WHEELBASE_LENGTH/Constants.WHEELBASE_WIDTH)-currentRobotHeading);
				y = -Constants.RADIUS_CENTER_TO_MODULE * Math.sin(Math.atan(Constants.WHEELBASE_LENGTH/Constants.WHEELBASE_WIDTH)-currentRobotHeading);
				/*/
				x = -Constants.RADIUS_CENTER_TO_MODULE*Math.cos(Math.toRadians(intake.getCurrentAngle())+Constants.ANGLE_FRONT_MODULE_CENTER);
				y = Constants.RADIUS_CENTER_TO_MODULE*Math.sin(Math.toRadians(intake.getCurrentAngle())+Constants.ANGLE_FRONT_MODULE_CENTER);
		/**/	break;
			case NINETY: //90
				y = -Constants.RADIUS_CENTER_TO_MODULE * Math.cos(Math.atan(Constants.WHEELBASE_LENGTH/Constants.WHEELBASE_WIDTH)-currentRobotHeading);
				x = Constants.RADIUS_CENTER_TO_MODULE * Math.sin(Math.atan(Constants.WHEELBASE_LENGTH/Constants.WHEELBASE_WIDTH)-currentRobotHeading);
/*/
				y = Constants.RADIUS_CENTER_TO_MODULE*Math.cos(Math.toRadians(intake.getCurrentAngle())+Constants.ANGLE_FRONT_MODULE_CENTER);
				x = Constants.RADIUS_CENTER_TO_MODULE*Math.sin(Math.toRadians(intake.getCurrentAngle())+Constants.ANGLE_FRONT_MODULE_CENTER);
		/**/	break;
			case ONE_EIGHTY: //180
				
				x = Constants.RADIUS_CENTER_TO_MODULE * Math.cos(Math.atan(Constants.WHEELBASE_LENGTH/Constants.WHEELBASE_WIDTH)-currentRobotHeading);
				y = Constants.RADIUS_CENTER_TO_MODULE * Math.sin(Math.atan(Constants.WHEELBASE_LENGTH/Constants.WHEELBASE_WIDTH)-currentRobotHeading);
/*/
				x = Constants.RADIUS_CENTER_TO_MODULE*Math.cos(Math.toRadians(intake.getCurrentAngle())+Constants.ANGLE_FRONT_MODULE_CENTER);
				y = -Constants.RADIUS_CENTER_TO_MODULE*Math.sin(Math.toRadians(intake.getCurrentAngle())+Constants.ANGLE_FRONT_MODULE_CENTER);
			/**/break;
			case TWO_SEVENTY: //270
				y = Constants.RADIUS_CENTER_TO_MODULE * Math.cos(Math.atan(Constants.WHEELBASE_LENGTH/Constants.WHEELBASE_WIDTH)-currentRobotHeading);
				x = -Constants.RADIUS_CENTER_TO_MODULE * Math.sin(Math.atan(Constants.WHEELBASE_LENGTH/Constants.WHEELBASE_WIDTH)-currentRobotHeading);
/*/
				y = -Constants.RADIUS_CENTER_TO_MODULE*Math.cos(Math.toRadians(intake.getCurrentAngle())+Constants.ANGLE_FRONT_MODULE_CENTER);
				x = -Constants.RADIUS_CENTER_TO_MODULE*Math.sin(Math.toRadians(intake.getCurrentAngle())+Constants.ANGLE_FRONT_MODULE_CENTER);
			/**/break;
			}
	// Second and third lines were not commented
			/**/
//			x = Constants.RADIUS_CENTER_TO_MODULE*Math.cos(Math.toRadians(intake.getCurrentAngle())+Constants.ANGLE_FRONT_MODULE_CENTER);
//			y = Constants.RADIUS_CENTER_TO_MODULE*Math.sin(Math.toRadians(intake.getCurrentAngle())+Constants.ANGLE_FRONT_MODULE_CENTER);
			/*/
			x = -(Constants.WHEELBASE_WIDTH/2)*Constants.DRIVE_TICKS_PER_INCH;
			y =  (Constants.WHEELBASE_LENGTH/2)*Constants.DRIVE_TICKS_PER_INCH;
			/**/
			/*x = 0;
			y = 0;
			findRobotX();
			findRobotY();
			x = 0-getRobotX();
			y = 0-getRobotY();
		*/}
	}
	
	// 2017-03-05 Added because of a potential logic error
	/** Checks whether all module angles are on target. */
	
	/**
	 * Updates the {@link Swerve} drivetrain's properties, including those of its component {@link SwerveDriveModule modules}.
	 * 
	 * <ul>
	 *  <li>{@link Swerve#refreshRobotHeading() Refreshes} its cached {@link Swerve#currentRobotHeading value} of the robot's heading
	 *  <li>Calls each Swerve module's member methods to
	 *   <ul>
	 *    <li>output its {@link SwerveDriveModule#debugValues() debug values}</li>
	 *    <li>update its {@link SwerveDriveModule#updateCoord() coordinates}</li>
	 *   </ul>
	 *  </li>
	 *  <li>{@link Swerve#findRobotCenter() Update} the coordinates of the robot's center</li>
	 *  <li></li>
	 * </ul>
	 * */
	public void initModules(){
		frontLeft.loadProperties();
		frontRight.loadProperties();
		rearLeft.loadProperties();
		rearRight.loadProperties();
	}
	public void update(){
		//if(!disableUpdates){
			//SmartDashboard.putNumber("xInput", xInput);
			//SmartDashboard.putNumber("yInput", yInput);
			//SmartDashboard.putNumber("rotation", rotateInput);
			refreshRobotHeading();
			
			frontRight.debugValues();
			frontLeft.debugValues();
			rearRight.debugValues();
			rearLeft.debugValues();
			
			frontRight.updateCoord();
			frontLeft.updateCoord();
			rearRight.updateCoord();
			rearLeft.updateCoord();
			
			SmartDashboard.putNumber("Module 1 Error", frontRight.rotationMotor.getSetpoint() - frontRight.getCurrentAngle());
			
			findRobotCenter();
			if(cyclesSinceLastUpdate > 10){

				AccelX = (getX() - lastXPos) - lastXVel;
				AccelY = (getY() - lastYPos) - lastYVel;
				lastXVel = getX() - lastXPos;
				lastYVel = getY() - lastYPos;
				lastXPos = getX();
				lastYPos = getY();
				//SmartDashboard.putNumber("X Velocity", lastXVel);
				//SmartDashboard.putNumber("Y Velocity", lastYVel);
				//SmartDashboard.putNumber("X Acceleration", AccelX);
				//SmartDashboard.putNumber("Y Acceleration", AccelY);
				cyclesSinceLastUpdate = 0;
			}
			cyclesSinceLastUpdate++;
			SmartDashboard.putNumber("Robot X (in) ", getRobotXInch());
			SmartDashboard.putNumber("Robot Y (in) ", getRobotYInch());
			//SmartDashboard.putNumber("Robot X (in) offsets", getXWithOffsets());
			//SmartDashboard.putNumber("Robot Y (in) offsets", getYWithOffsets());
			//SmartDashboard.putNumber("WhlToRbtX", Constants.RADIUS_CENTER_TO_MODULE * Math.cos(Math.atan(Constants.WHEELBASE_LENGTH/Constants.WHEELBASE_WIDTH)-currentRobotHeading)/Constants.DRIVE_TICKS_PER_INCH);
			//SmartDashboard.putNumber("WhlToRbtY", Constants.RADIUS_CENTER_TO_MODULE * Math.sin(Math.atan(Constants.WHEELBASE_LENGTH/Constants.WHEELBASE_WIDTH)-currentRobotHeading)/Constants.DRIVE_TICKS_PER_INCH);
			
	//		SmartDashboard.putNumber("Target Heading", _targetAngle);
	//		SmartDashboard.putNumber("turnErr", Util.boundAngleNeg180to180Degrees(Util.boundAngle0to360Degrees(intake.getCurrentAngle())-_targetAngle)); // add bounding to [-180,180]
			SmartDashboard.putNumber("Y Input", yInput);
			SmartDashboard.putNumber("X Input", xInput);
			SmartDashboard.putNumber("Rotation Input", rotateInput);
			if(xInput == 0 && yInput == 0 && rotateInput == 0){
				frontLeft.stopDriveMotor();
				frontRight.stopDriveMotor();
				rearLeft.stopDriveMotor();
				rearRight.stopDriveMotor();
			} else {
				frontLeft.disableBreakMode();
				frontRight.disableBreakMode();
				rearLeft.disableBreakMode();
				rearRight.disableBreakMode();
				double A = xInput - rotateInput * (Constants.WHEELBASE_LENGTH / Constants.SWERVE_R);
		        double B = xInput + rotateInput * (Constants.WHEELBASE_LENGTH / Constants.SWERVE_R);
		        double C = yInput - rotateInput * (Constants.WHEELBASE_WIDTH / Constants.SWERVE_R);
		        double D = yInput + rotateInput * (Constants.WHEELBASE_WIDTH / Constants.SWERVE_R);
		        
		        double frontRightWheelSpeed = Math.sqrt((B * B) + (C * C));
		        double frontLeftWheelSpeed  = Math.sqrt((B * B) + (D * D));
		        double rearLeftWheelSpeed   = Math.sqrt((A * A) + (D * D));
		        double rearRightWheelSpeed  = Math.sqrt((A * A) + (C * C));
		       
		        double max = frontRightWheelSpeed;
		        //System.out.println("xInput:" + xInput + " yInput:" + yInput + " rot:" + rotateInput + " max: " + max);
		        max = Util.normalize(max, frontLeftWheelSpeed);
		        max = Util.normalize(max, rearLeftWheelSpeed);
		        max = Util.normalize(max, rearRightWheelSpeed);
		        if(max > 1.0){
		        	frontRightWheelSpeed /= max;
		            frontLeftWheelSpeed /= max;
		            rearLeftWheelSpeed /= max;
		            rearRightWheelSpeed /= max;
		        }
		        
		       
		        
		        double frontRightSteeringAngle = Math.atan2(B, C)*180/Math.PI; 
		        double frontLeftSteeringAngle = Math.atan2(B, D)*180/Math.PI;
		        double rearLeftSteeringAngle = Math.atan2(A, D)*180/Math.PI;
		        double rearRightSteeringAngle = Math.atan2(A, C)*180/Math.PI;
		        
		        /*
		        double inverseAngle = Util.boundAngle0to360Degrees(frontRightSteeringAngle - 180);
		        if(Math.abs(frontRight.getCurrentAngle() - inverseAngle) < Math.abs(frontRight.getCurrentAngle() - frontRightSteeringAngle)){
		        	frontRightSteeringAngle = inverseAngle;
		        	frontRightWheelSpeed = -frontRightWheelSpeed;
		        }*/
		        
				/**if(SmartDashboard.getBoolean("Manual Wheel Headings?", false)) {
					frontRightSteeringAngle = SmartDashboard.getNumber("Manual Heading 1", 0); 
			        frontLeftSteeringAngle = SmartDashboard.getNumber("Manual Heading 2", 0);
			        rearLeftSteeringAngle = SmartDashboard.getNumber("Manual Heading 3", 0);
			        rearRightSteeringAngle = SmartDashboard.getNumber("Manual Heading 4", 0);
				}/**/
		        
		        frontLeft.setGoal(frontLeftSteeringAngle);
				frontRight.setGoal(frontRightSteeringAngle);
				rearLeft.setGoal(rearLeftSteeringAngle);
				rearRight.setGoal(rearRightSteeringAngle);
				/**
				 * 2017-03-05 Added because of a potential logic error:
				 * <p>
				 * 	Previously, each module checked independently whether its angle was on target,
				 *   then applied power to its own drive motor if it was. Suppose that modules 1 and 2
				 *   are on target, but 3 and 4 are not. 1 and 2 will apply power, moving the robot and
				 *   the other modules. In the meantime, 3 and 4 do not apply power, but get moved
				 *   around and thrown off somewhat.</p>
				 * <p>Proposed solution: Check whether all module angles are on target. If they are,
				 *   apply power to each of them. (Note that the {@link SubSystems.Swerve.SwerveDriveModule#setDriveSpeed() setDriveSpeed}
				 *   method checks the angle status another time before applying power.) If they are not,
				 *   do not apply power to any of the modules' drive motors.</p>
				 * */
					frontLeft.setDriveSpeed(frontLeftWheelSpeed);
					frontRight.setDriveSpeed(-frontRightWheelSpeed);
					rearLeft.setDriveSpeed(rearLeftWheelSpeed);
					rearRight.setDriveSpeed(-rearRightWheelSpeed);
				//Util.sdSimpleClosedLoop("Heading", "Angle", currentRobotHeading, _targetAngle);
				
//				SmartDashboard.putNumber(" Heading Angle ", currentRobotHeading); // imported from Intake
				SmartDashboard.putNumber(" Heading Set Point ", _targetAngle); // added from Swerve.sendInput()
				SmartDashboard.putNumber(" Heading Error ", getError());
				
			}
		//}
	}	
	
	double Cap(double value, double peak) {
		if (value < -peak)
			return -peak;
		if (value > peak)
			return peak;
		return value;
	}
	
	double MaxCorrection(double forwardThrot, double scalor) {
		if(forwardThrot < 0) {forwardThrot = -forwardThrot;}
		forwardThrot *= scalor;
		if(forwardThrot < 0.20)
			return 0.20;
		return forwardThrot;
	}
	
/*	void UpdatGains() {
		kPgain = _spareTalon.getP();
		kDgain = _spareTalon.getD();
		kMaxCorrectionRatio = _spareTalon.getF();
	}*/
	public void resetCoord(AnglePresets i){		
		disableUpdates = true;
		robotX = 0;
		robotY = 0;	
		rearRight.resetCoord(i);
		robotX = 0;
		robotY = 0;				
		frontRight.resetFollower();
		disableUpdates = false;
	}
	public boolean headingOnTarget(){
		if(Math.abs(getError()) < Constants.HEADING_MAX_ERROR)
			onTarget--;
		else
			onTarget = Constants.MIN_CYCLES_HEADING_ON_TARGET;
		return onTarget <= 0;
	}
	public boolean isImpacting(){
		return rearRight.driveMotor.getOutputCurrent() > Constants.SWERVE_IMPACT_CURRENT_THRESHOLD;
	}
	public double getError(){
		return _targetAngle - intake.getCurrentAngle();
	}
	public void updateCoord(){
		/*frontLeft.updateCoord();
		double tempX = frontLeft.getX() + xOffset;
		double tempY = frontLeft.getY() - yOffset;
		double dx = distanceTravelled * Math.cos(Math.toRadians(intake.getCurrentAngle()));
        double dy = distanceTravelled * Math.sin(Math.toRadians(intake.getCurrentAngle()));
        x += dx;
        y += dy;*/
	}
	public double getXVelocity(){
		return lastXVel;
	}
	public double getYVelocity(){
		return lastYVel;
	}
	private boolean cannotReverse = true;
	public void enableReverse(){
		cannotReverse = false;
		frontRight.enableReverse();
		frontLeft.enableReverse();
		rearRight.enableReverse();
		rearLeft.enableReverse();
	}
	public void disableReverse(){
		cannotReverse = true;
		frontRight.disableReverse();
		frontLeft.disableReverse();
		rearRight.disableReverse();
		rearLeft.disableReverse();
	}
	public void stopRotating(){
		frontRight.stopRotation();
		frontLeft.stopRotation();
		rearLeft.stopRotation();
		rearRight.stopRotation();
	}
	public void setTeleRampRate(){
		frontRight.setTeleRampRate();
		frontLeft.setTeleRampRate();
		rearRight.setTeleRampRate();
		rearLeft.setTeleRampRate();
	}
	public void setAutoRampRate(){
		frontRight.setAutoRamprate();
		frontLeft.setAutoRamprate();
		rearRight.setAutoRamprate();
		rearLeft.setAutoRamprate();
	}
}