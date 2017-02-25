package SubSystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import Utilities.Constants;
import Utilities.Ports;
import Utilities.Util;
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
	private int onTarget = 0;
	
	public SwerveDriveModule frontLeft;
	private SwerveDriveModule frontRight;
	private SwerveDriveModule rearLeft;
	private SwerveDriveModule rearRight;
	
	
	public enum AnglePresets{
		ZERO,NINETY,ONE_EIGHTY, TWO_SEVENTY
	}
	/*
	//private double x = 0.0;
	//private double y = 0.0;
	//private double xOffset = 8.0;
	//private double yOffset = 11.0;
	/**/
	//Pigeon IMU Data
	
	
	public void setHeading(double goal,boolean rotation){		
		if(rotation)
			headingController = HeadingController.Rotation;
		_targetAngle = Util.continousAngle(goal,intake.getCurrentAngle());
	}
	enum HeadingController{
		Off, Heading,Rotation, Reset 
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
		double adjust = intake.getCurrentAngle() - Vision.getAngle();
    	setHeading(adjust,false);
	}
	
	
	 /* cap corrective turning throttle to 30 percent of forward throttle */
	
	private double a;// = intake.getCurrentAngle();
	private double robotX;
	private double robotY;
	private void refreshRobotAngle() {a = Math.toRadians(intake.getCurrentAngle());}
	private void findRobotX() {robotX = frontLeft.getX()+ (Constants.RADIUS_CENTER_TO_MODULE * Math.cos(a+Constants.ANGLE_FRONT_MODULE_CENTER));}
	private void findRobotY() {robotY = frontLeft.getY()- (Constants.RADIUS_CENTER_TO_MODULE * Math.sin(a+Constants.ANGLE_FRONT_MODULE_CENTER));}
	public double getRobotX() {return robotX;}
	public double getRobotY() {return robotY;}
	public double getRobotXInch() {return robotX/Constants.DRIVE_TICKS_PER_INCH;}
	public double getRobotYInch() {return robotY/Constants.DRIVE_TICKS_PER_INCH;}
	/**/
	
	
	
	public void sendInput(double x, double y, double rotateX,double rotateY,boolean halfPower,boolean robotCentric,boolean moonManeuver){
		
//		SmartDashboard.putNumber("X Stick", rotateX);
//		SmartDashboard.putNumber("Y Stick", rotateY);
		
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
			double angleDiff = Math.abs(_targetAngle - intake.getCurrentAngle());
			switch(headingController){
			case Off:
				rotationCorrection = 0.0;
				break;
			case Heading:
				if(angleDiff > 0.5){
					rotationCorrection = (_targetAngle - intake.getCurrentAngle()) * Constants.SWERVE_HEADING_GAIN_P - (intake.currentAngularRate) * Constants.SWERVE_HEADING_GAIN_D;
					rotationCorrection = Cap(rotationCorrection, Constants.SWERVE_MAX_CORRECTION_HEADING);
				}else{
				rotationCorrection = 0.0;
				}
				break;
			case Rotation:				
				if(angleDiff < 8){
					rotationCorrection = (_targetAngle - intake.getCurrentAngle()) * Constants.SWERVE_SMALL_TURNING_GAIN_P - (intake.currentAngularRate) * Constants.SWERVE_SMALL_TURNING_GAIN_D;
					rotationCorrection = Cap(rotationCorrection, Constants.SWERVE_SMALL_HEADING_MAX_CORRECTION_RATIO);
				}else{
					rotationCorrection = (_targetAngle - intake.getCurrentAngle()) * Constants.SWERVE_TURNING_GAIN_P - (intake.currentAngularRate) * Constants.SWERVE_TURNING_GAIN_D;
					rotationCorrection = Cap(rotationCorrection, Constants.SWERVE_HEADING_MAX_CORRECTION_RATIO);
				}
				if(angleDiff < Constants.SWERVE_ROTATION_HEADING_ON_TARGET_THRESHOLD){
					rotationOnTarget--;
					if(rotationOnTarget <= 0){ // `rotationOnTarget' was `onTarget'
						headingController = HeadingController.Heading;
					}
				}else{
					rotationOnTarget = Constants.MIN_CYCLES_HEADING_ON_TARGET;
				}
				break;
			case Reset:
				
				break;
			default:
				
				break;
			}
			if (headingController == HeadingController.Rotation) {
				
//				SmartDashboard.putNumber("ROTATE_CORRECT", rotationCorrection);
			} else if (headingController == HeadingController.Off) {
				rotationCorrection = 0;
			} else {
			}
			if(halfPower){
				y = y * 0.5;
				x = x * 0.5;			
			}else{
				y = y * 1.0;
				x = x * 1.0;
			}
			if(x == 0.0 && y == 0.0){
				rotateInput = rotateInput + rotationCorrection;
			}else{
				if(halfPower)
					rotateInput = (rotateInput * Constants.SWERVE_ROTATION_SCALE_FACTOR) + rotationCorrection;
				else
					rotateInput = (rotateInput * Constants.SWERVE_ROTATION_SCALE_FACTOR_FAST) + rotationCorrection;
			}		
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
		SmartDashboard.putNumber("Target Heading", _targetAngle);
		switch(headingController){
		case Off:
			SmartDashboard.putString("Heading Controller Test", "OFF");
			break;
		case Rotation:
			SmartDashboard.putString("Heading Controller Test", "rotation");
			break;
		case Heading:
			SmartDashboard.putString("Heading Controller Test", "Heading");
			break;
		case Reset:
			SmartDashboard.putString("Heading Controller Test", "reset");
			break;
			default:
				SmartDashboard.putString("Heading Controller Test", "default");
				break;
		}
	}
	
	public class SwerveDriveModule{
		private CANTalon rotationMotor;
		public CANTalon driveMotor;
		private int moduleID;
		private int absolutePosition,absolutePosition2;
		private double x = 0.0;
		private double y = 0.0;
		private double offSet = 0.0;
		
		private double lastEncPosition = 0.0;
		private double currentEncPosition;
		private double currentIntakeAngle;
		private double currentModuleAngle;
		
		private double getCurrentEncPosition() {return currentEncPosition;}
		private double getCurrentIntakeAngle() {return currentIntakeAngle;}
		private double getCurrentModuleAngle() {return currentModuleAngle;}
		private void setCurrentEncPosition(double _currentEncPosition) {currentEncPosition = _currentEncPosition;}
		private void setCurrentIntakeAngle(double _currentIntakeAngle) {currentIntakeAngle = _currentIntakeAngle;}
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
			return getCurrentEncPosition() - relativeTickZero;
		}
		
	    
		
		
		public void updateCoord(){
			setCurrentEncPosition(driveMotor.getEncPosition());
			setCurrentIntakeAngle(intake.getCurrentAngle());
			setCurrentModuleAngle(rotationMotor.get()-offSet);
//			SmartDashboard.putString("updateCoord():setCurrentModule"+Integer.toString(moduleID)+"Angle", Double.toString(Util.boundAngle0to360Degrees(getCurrentModuleAngle()))+" / "+Double.toString(Util.boundAngle0to360Degrees(rotationMotor.get())));
			double distanceTravelled = -(getCurrentEncPosition()-lastEncPosition);// * Constants.DRIVE_INCHES_PER_CLICK; //0.00180143;*Constants.DRIVE_CLICKS_PER_INCH; // inches
	        totalDistanceTravelled -= distanceTravelled; // inches
//	        SmartDashboard.putNumber("distanceTravelled"+Integer.toString(moduleID), distanceTravelled);
//	        SmartDashboard.putBoolean(Integer.toString(moduleID) + " Travelling ", isTravelling());
//	        SmartDashboard.putNumber(Integer.toString(moduleID) + " Distance (in) ", totalDistanceTravelled);
//	        SmartDashboard.putNumber(Integer.toString(moduleID) + " cos = ", Math.cos(Math.toRadians(360-rotationMotor.get()+90)));
//	        SmartDashboard.putNumber(Integer.toString(moduleID) + " sin = ", Math.sin(Math.toRadians(360-rotationMotor.get()+90)));
	        if(!isRotating()){
		        double dx = distanceTravelled * Math.cos(Math.toRadians(getCurrentIntakeAngle()/*-360/**/+getCurrentModuleAngle()+90));
		        double dy = -distanceTravelled * Math.sin(Math.toRadians(getCurrentIntakeAngle()/*-360/**/+getCurrentModuleAngle()+90));
		        x += dx;
		        y += dy;
	        }
//	        SmartDashboard.putNumber("DRV_X" + Integer.toString(moduleID), x);
//	        SmartDashboard.putNumber("DRV_Y" + Integer.toString(moduleID), y);
//	        SmartDashboard.putNumber(Integer.toString(moduleID) + " dX ", dx*1000);
//	        SmartDashboard.putNumber(Integer.toString(moduleID) + " dY ", dy*1000);
	        SmartDashboard.putNumber(Integer.toString(moduleID) + " X (tick) ", x);
	        SmartDashboard.putNumber(Integer.toString(moduleID) + " Y (tick) ", y);
	        SmartDashboard.putNumber(Integer.toString(moduleID) + " X (in) ", x/Constants.DRIVE_TICKS_PER_INCH);
	        SmartDashboard.putNumber(Integer.toString(moduleID) + " Y (in) ", y/Constants.DRIVE_TICKS_PER_INCH);
	        
			lastEncPosition = getCurrentEncPosition();
		}
		public double getX(){return x;}
		public double getY(){return y;}
		public double getXInch(){return x/Constants.DRIVE_TICKS_PER_INCH;}
		public double getYInch(){return y/Constants.DRIVE_TICKS_PER_INCH;}
		public void resetCoord(AnglePresets i){
			x = 0;
			y = 0;
			driveMotor.setEncPosition(0);
			setRotationOffsetClicks(0);
			setTotalDistanceTravelled(0);
			lastEncPosition = 0;
			setCurrentEncPosition(0);
			relativeTickZero = getCurrentEncPosition();
			initModule(i);
		}
		public void debugValues(){
			if(wheelError() >= Constants.TURNING_DETECT_THRESHOLD) {isRotating = true;} else {isRotating = false;}
			if(isRotating || Math.abs(getCurrentEncPosition()-lastEncPosition) <= Constants.DRIVING_DETECT_THRESHOLD /*clicks*/) {isTravelling = false;} else {isTravelling = true;}
			//Note #3
//			SmartDashboard.putNumber(Integer.toString(moduleID) + " Rotation Angle (deg) ", Util.boundAngle0to360Degrees(getCurrentModuleAngle()));
//			SmartDashboard.putNumber("DRV_" + Integer.toString(moduleID), driveMotor.get());
			SmartDashboard.putNumber("OFF_" + Integer.toString(moduleID), Util.boundAngle0to360Degrees(getCurrentAngle()/*-offSet*/));
//			SmartDashboard.putNumber("GOAL " + Integer.toString(moduleID), Util.boundAngle0to360Degrees(rotationMotor.getSetpoint()-(360-offSet)));
//			SmartDashboard.putNumber("W_ERR" + Integer.toString(moduleID), Util.boundAngle0to360Degrees(wheelError()));
			SmartDashboard.putNumber(Integer.toString(moduleID)+ " Raw Encoder Clicks ", relativeTickCount()/*getCurrentEncPosition()*/);
//			SmartDashboard.putBoolean(Integer.toString(moduleID) + " Rotating ", isRotating());
//			SmartDashboard.putNumber(Integer.toString(moduleID) + " Rotation Clicks ", getRotationOffsetClicks());
//			SmartDashboard.putNumber(Integer.toString(moduleID) + " Driven Clicks ", getCurrentEncPosition() - getRotationOffsetClicks());
			SmartDashboard.putNumber(Integer.toString(moduleID) + "Inches since last zero: ", getCurrentEncPosition()/Constants.DRIVE_TICKS_PER_INCH);//Constants.DRIVE_INCHES_PER_CLICK);
		}
		public SwerveDriveModule(int rotationMotorPort, int driveMotorPort,int moduleNum,double _offSet){
			rotationMotor = new CANTalon(rotationMotorPort);
			rotationMotor.setPID(4, 0.0, 20, 0.0, 0, 0.0, 0);
			driveMotor = new CANTalon(driveMotorPort);
			absolutePosition2 = driveMotor.getPulseWidthPosition() & 0xFFF;
	    	driveMotor.setEncPosition(absolutePosition2);
	    	driveMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
	    	driveMotor.reverseSensor(false);
	    	driveMotor.configEncoderCodesPerRev(360);
	    	driveMotor.configNominalOutputVoltage(+0f, -0f);
	    	driveMotor.configPeakOutputVoltage(+12f, 0);
	    	driveMotor.setAllowableClosedLoopErr(0); 
//	    	driveMotor.changeControlMode(TalonControlMode.);
//	    	driveMotor.set(driveMotor.getPosition());
			loadProperties();
			moduleID = moduleNum;        
			offSet = _offSet;
		}
		
		public void setGoal(double goalAngle)
	    {			
			rotationMotor.set(Util.continousAngle(goalAngle-(360-offSet),getCurrentAngle()));
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
	    	rotationMotor.configPeakOutputVoltage(+6f, -6f);
	    	rotationMotor.setAllowableClosedLoopErr(0); 
	    	rotationMotor.changeControlMode(TalonControlMode.Position);
	    	rotationMotor.setPID(4, 0.0, 0.0, 0.00, 0, 0.0, 0);
	    	rotationMotor.setProfile(0);
	    	rotationMotor.set(rotationMotor.get());
	    }
	   
	    public void setDriveSpeed(double power){
	    	// This function determines whether we're close enough to a wheel's desired angle to start driving that wheel
	    	if(wheelError() < Constants.TURNING_ADD_POWER_THRESHOLD) {
	    		driveMotor.set(-power);}
	   	}
	    public void stopDriveMotor(){
	    	driveMotor.set(0);
	    }
		public double getCurrentAngle(){
			return rotationMotor.get();
		}
		public void initModule(AnglePresets i){
			switch(i){
			case ZERO: //0
				x = -Constants.RADIUS_CENTER_TO_MODULE*Math.cos(Math.toRadians(intake.getCurrentAngle())+Constants.ANGLE_FRONT_MODULE_CENTER);
				y = Constants.RADIUS_CENTER_TO_MODULE*Math.sin(Math.toRadians(intake.getCurrentAngle())+Constants.ANGLE_FRONT_MODULE_CENTER);
				break;
			case NINETY: //90
				y = Constants.RADIUS_CENTER_TO_MODULE*Math.cos(Math.toRadians(intake.getCurrentAngle())+Constants.ANGLE_FRONT_MODULE_CENTER);
				x = Constants.RADIUS_CENTER_TO_MODULE*Math.sin(Math.toRadians(intake.getCurrentAngle())+Constants.ANGLE_FRONT_MODULE_CENTER);
				break;
			case ONE_EIGHTY: //180
				x = Constants.RADIUS_CENTER_TO_MODULE*Math.cos(Math.toRadians(intake.getCurrentAngle())+Constants.ANGLE_FRONT_MODULE_CENTER);
				y = -Constants.RADIUS_CENTER_TO_MODULE*Math.sin(Math.toRadians(intake.getCurrentAngle())+Constants.ANGLE_FRONT_MODULE_CENTER);
				break;
			case TWO_SEVENTY: //270
				y = -Constants.RADIUS_CENTER_TO_MODULE*Math.cos(Math.toRadians(intake.getCurrentAngle())+Constants.ANGLE_FRONT_MODULE_CENTER);
				x = -Constants.RADIUS_CENTER_TO_MODULE*Math.sin(Math.toRadians(intake.getCurrentAngle())+Constants.ANGLE_FRONT_MODULE_CENTER);
				break;
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
	
	public void update(){
		frontRight.debugValues();
		frontLeft.debugValues();
		rearRight.debugValues();
		rearLeft.debugValues();
		
		frontRight.updateCoord();
		frontLeft.updateCoord();
		rearRight.updateCoord();
		rearLeft.updateCoord();
		
		refreshRobotAngle();
		findRobotX();
		findRobotY();
		SmartDashboard.putNumber("Robot X", getRobotX());
		SmartDashboard.putNumber("Robot Y", getRobotY());

		SmartDashboard.putNumber("Robot X (in) ", getRobotXInch());
		SmartDashboard.putNumber("Robot Y (in) ", getRobotYInch());
		
//		SmartDashboard.putNumber("Target Heading", _targetAngle);
//		SmartDashboard.putNumber("turnErr", Util.boundAngleNeg180to180Degrees(Util.boundAngle0to360Degrees(intake.getCurrentAngle())-_targetAngle)); // add bounding to [-180,180]
		if(xInput == 0 && yInput == 0 && rotateInput == 0){
			frontLeft.stopDriveMotor();
			frontRight.stopDriveMotor();
			rearLeft.stopDriveMotor();
			rearRight.stopDriveMotor();
		}
		
		else{
			double A = xInput - rotateInput * (Constants.WHEELBASE_LENGTH / Constants.SWERVE_R);
	        double B = xInput + rotateInput * (Constants.WHEELBASE_LENGTH / Constants.SWERVE_R);
	        double C = yInput - rotateInput * (Constants.WHEELBASE_WIDTH / Constants.SWERVE_R);
	        double D = yInput + rotateInput * (Constants.WHEELBASE_WIDTH / Constants.SWERVE_R);
	        
	        double frontRightWheelSpeed = Math.sqrt((B * B) + (C * C));
	        double frontLeftWheelSpeed  = Math.sqrt((B * B) + (D * D));
	        double rearLeftWheelSpeed   = Math.sqrt((A * A) + (D * D));
	        double rearRightWheelSpeed  = Math.sqrt((A * A) + (C * C));
	       
	        double max = frontRightWheelSpeed;
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
			
			frontLeft.setDriveSpeed(frontLeftWheelSpeed);
			frontRight.setDriveSpeed(-frontRightWheelSpeed);
			rearLeft.setDriveSpeed(rearLeftWheelSpeed);
			rearRight.setDriveSpeed(-rearRightWheelSpeed);
		}
	}	
	
	double Cap(double value, double peak) {
		if (value < -peak)
			return -peak;
		if (value > +peak)
			return +peak;
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
		frontLeft.resetCoord(i);
		robotX = 0;
		robotY = 0;
	}
	public boolean headingOnTarget(){
		if(Math.abs(_targetAngle - intake.getCurrentAngle()) < Constants.HEADING_MAX_ERROR)
			onTarget--;
		else
			onTarget = Constants.MIN_CYCLES_HEADING_ON_TARGET;
		return onTarget <= 0;
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
}