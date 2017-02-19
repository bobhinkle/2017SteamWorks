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
	
	public SwerveDriveModule frontLeft;
	private SwerveDriveModule frontRight;
	private SwerveDriveModule rearLeft;
	private SwerveDriveModule rearRight;
	private static double R = 27.16;
	private double rotationScaleFactor = .225;
	private double rotationScaleFactorFast = 0.35;
	private double xInput,yInput,rotateInput = 0;
	//Pigeon IMU Data
	
	boolean tracking = false;
	//Swerve Turning Gains
	double kPgain = 0.02; /* percent throttle per degree of error */ //0.02
	double kDgain = 0.00425; /* percent throttle per angular velocity dps */ //0.00425
	double kPgainSmall = 0.015; //0.015
	double kDgainSmall = 0.002; //0.002
	double kMaxCorrectionRatio = 0.75; //0.75
	double kMaxCorrectionRatioSmall = 0.18; //0.18
	 /* cap corrective turning throttle to 30 percent of forward throttle */
	
	double _targetAngle = 0.0;
	private double allowableError = 2.0;
	double rotationCorrection;
	int _printLoops = 0;
	private Intake intake;
	private int onTargetThresh = 30;
	private int onTarget = 0;
	public void setHeading(double goal){
		_targetAngle = continousAngle2(goal,intake.getCurrentAngle());
	}
	public double continousAngle2(double goal, double current){
		double BGA = Util.boundAngle0to360Degrees(goal);			
		double CA = current;
		double BCA = Util.boundAngle0to360Degrees(CA);
		double OA = BCA - 180.0;
		double DA  = OA - BGA;
		if(DA < -360){
			DA = DA + 360;
		}
		if(DA > 0.0){
			return CA + 180.0 - Math.abs(DA);
		}else{
			return CA - 180.0 + Math.abs(DA);
		}
	}
	enum HeadingController{
		Off, On, Reset
	}
	public HeadingController headingController = HeadingController.Off;
	
	public Swerve(){
		frontLeft  = new SwerveDriveModule(Ports.FRONT_LEFT_ROTATION,Ports.FRONT_LEFT_DRIVE,2,Constants.FRONT_LEFT_TURN_OFFSET);
		frontRight = new SwerveDriveModule(Ports.FRONT_RIGHT_ROTATION,Ports.FRONT_RIGHT_DRIVE,1,Constants.FRONT_RIGHT_TURN_OFFSET);
		rearLeft   = new SwerveDriveModule(Ports.REAR_LEFT_ROTATION,Ports.REAR_LEFT_DRIVE,3,Constants.REAR_LEFT_TURN_OFFSET);
		rearRight  = new SwerveDriveModule(Ports.REAR_RIGHT_ROTATION,Ports.REAR_RIGHT_DRIVE,4,Constants.REAR_RIGHT_TURN_OFFSET);
		intake = Intake.getInstance();		
	}
	public static Swerve getInstance()
    {
        if( instance == null )
            instance = new Swerve();
        return instance;
    }
	
	public void swerveTrack(){
		double adjust = intake.getCurrentAngle() - Vision.getAngle();
    	setHeading(adjust);
    	tracking = true;
	}
	public void sendInput(double x, double y, double rotateX,double rotateY,boolean halfPower,boolean robotCentric,boolean moonManeuver){
		
		tracking = false;
		SmartDashboard.putNumber("X Stick", rotateX);
		SmartDashboard.putNumber("Y Stick", rotateY);
		if(moonManeuver){
			headingController = HeadingController.Off;
			yInput = 0.0;
			xInput = -(rotateX * 0.2) * ((Math.abs(y)*2)+1) ;
			rotateInput = rotateX * 0.2;
			SmartDashboard.putNumber("xMoon", xInput);
			SmartDashboard.putNumber("rMoon", rotateInput);
		}
		else{
			if(Math.abs(rotateX) > 0.1){
				headingController = HeadingController.Off;
				rotateInput = rotateX;
			}else if(rotateX > -0.1 && rotateX < 0.1 && headingController == HeadingController.Off){
				headingController = HeadingController.On;
				_targetAngle = intake.getCurrentAngle();
			}else{
				rotateInput = 0.0;
			}
			double angle = intake.getCurrentAngle()/180.0*Math.PI;
			if (headingController == HeadingController.On) {
				double angleDiff = Math.abs(_targetAngle - intake.getCurrentAngle());
				if(angleDiff > 0.5){
					if(angleDiff < 4){
						rotationCorrection = (_targetAngle - intake.getCurrentAngle()) * kPgainSmall - (intake.currentAngularRate) * kDgainSmall;
						rotationCorrection = Cap(rotationCorrection, kMaxCorrectionRatioSmall);
					}else{
						rotationCorrection = (_targetAngle - intake.getCurrentAngle()) * kPgain - (intake.currentAngularRate) * kDgain;
						rotationCorrection = Cap(rotationCorrection, kMaxCorrectionRatio);
					}
				}else{
					rotationCorrection = 0.0;
				}
			
				SmartDashboard.putNumber("ROTATE_CORRECT", rotationCorrection);
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
					rotateInput = (rotateInput * rotationScaleFactor) + rotationCorrection;
				else
					rotateInput = (rotateInput * rotationScaleFactorFast) + rotationCorrection;
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
	}
	
	public class SwerveDriveModule{
		private CANTalon rotationMotor;
		public CANTalon driveMotor;
		private int moduleID;
		private int absolutePosition,absolutePosition2;
		private double x = 0.0;
		private double y = 0.0;
		private double offSet = 0.0;  // offSet of what?
		private double lastDistance = 0.0;
		
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
		
		public double continousAngle(double goal, double current){
			double BGA = Util.boundAngle0to360Degrees(goal);			
			double CA = current;
			double BCA = Util.boundAngle0to360Degrees(CA);
			double OA = BCA - 180.0;
			double DA  = OA - BGA;
			if(DA < -360){
				DA = DA + 360;
			}
			if(DA > 0.0){
				return CA + 180.0 - Math.abs(DA);
			}else{
				return CA - 180.0 + Math.abs(DA);
			}
		}
		public void updateCoord(){/**
			if(isRotating()) {
				rotationOffsetClicks += driveMotor.getEncPosition()-lastDistance; // clicks
			} else if(intake.getCurrentAngularRate() >= Constants.ROBOT_ROTATING_DETECT_THRESHOLD) {
				robotRotationOffsetClicks += driveMotor.getEncPosition()-lastDistance;
			} else {/**/
		        double distanceTravelled = -(driveMotor.getEncPosition()-lastDistance) * Constants.DRIVE_CLICKS_PER_INCH; //0.00180143;*Constants.DRIVE_CLICKS_PER_INCH; // inches
		        totalDistanceTravelled -= distanceTravelled; // inches
//		        if (Math.abs(distanceTravelled) < 0.0005) {isTravelling = false;} else {isTravelling = true;}
		        SmartDashboard.putBoolean(Integer.toString(moduleID) + " Travelling ", isTravelling());
		        SmartDashboard.putNumber(Integer.toString(moduleID) + " Distance (in) ", totalDistanceTravelled);
		        SmartDashboard.putNumber(Integer.toString(moduleID) + " cos = ", Math.cos(Math.toRadians(360-rotationMotor.get()+90)));
		        SmartDashboard.putNumber(Integer.toString(moduleID) + " sin = ", Math.sin(Math.toRadians(360-rotationMotor.get()+90)));
		        double dx = distanceTravelled * Math.cos(Math.toRadians(360-rotationMotor.get()+90));
		        double dy = distanceTravelled * Math.sin(Math.toRadians(360-rotationMotor.get()+90));
		        x += dx;
		        y += dy;
//		        SmartDashboard.putNumber("DRV_X" + Integer.toString(moduleID), x);
//		        SmartDashboard.putNumber("DRV_Y" + Integer.toString(moduleID), y);
		        SmartDashboard.putNumber(Integer.toString(moduleID) + " dX ", dx*1000);
		        SmartDashboard.putNumber(Integer.toString(moduleID) + " dY ", dy*1000);
		        SmartDashboard.putNumber(Integer.toString(moduleID) + " X (in) ", x);
		        SmartDashboard.putNumber(Integer.toString(moduleID) + " Y (in) ", y);
		//	}
			lastDistance = driveMotor.getEncPosition();
		}
		public double getX(){
			return x;
		}
		public double getY(){
			return y;
		}
		public void resetCoord(){
			x = 0;
			y = 0;
			driveMotor.setEncPosition(0);
			setRotationOffsetClicks(0);
			setTotalDistanceTravelled(0);
		}
		public void debugValues(){
			if(wheelError() >= Constants.TURNING_DETECT_THRESHOLD) {isRotating = true;} else {isRotating = false;}
			if(isRotating || Math.abs(driveMotor.getEncPosition()-lastDistance) <= Constants.DRIVING_DETECT_THRESHOLD /*clicks*/) {isTravelling = false;} else {isTravelling = true;}
			//Note #3
			SmartDashboard.putNumber(Integer.toString(moduleID) + " Rotation Angle (deg) ", Util.boundAngle0to360Degrees(rotationMotor.get()));
//			SmartDashboard.putNumber("DRV_" + Integer.toString(moduleID), driveMotor.get());
//			SmartDashboard.putNumber("OFF " + Integer.toString(moduleID), rotationMotor.get()-(360-offSet));
//			SmartDashboard.putNumber("GOAL " + Integer.toString(moduleID), rotationMotor.getSetpoint());
			SmartDashboard.putNumber("W_ERR" + Integer.toString(moduleID), Util.boundAngle0to360Degrees(wheelError()));
			SmartDashboard.putNumber(Integer.toString(moduleID)+ " Raw Encoder Clicks ", driveMotor.getEncPosition());
			SmartDashboard.putBoolean(Integer.toString(moduleID) + " Rotating ", isRotating());
			SmartDashboard.putNumber(Integer.toString(moduleID) + " Rotation Clicks ", getRotationOffsetClicks());
			SmartDashboard.putNumber(Integer.toString(moduleID) + " Driven Clicks ", driveMotor.getEncPosition() - getRotationOffsetClicks());
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
			resetCoord();
		}
		
		public void setGoal(double goalAngle)
	    {
			rotationMotor.set(continousAngle(goalAngle-(360-offSet),getCurrentAngle()));
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
	    	rotationMotor.setPID(8, 0.0, 0.0, 0.00, 0, 0.0, 0);
	    	rotationMotor.setProfile(0);
	    	rotationMotor.set(rotationMotor.get());
	    }
	   
	    public void setDriveSpeed(double power){
	    	if(wheelError() < 10)
	    		driveMotor.set(-power);	    
	   	}
	    public void stopDriveMotor(){
	    	driveMotor.set(0);
	    }
		public double getCurrentAngle(){
			return rotationMotor.get();
		}
		
	}
	
	public void update(){
		frontRight.debugValues();
		frontLeft.debugValues();
		rearRight.debugValues();
		rearLeft.debugValues();
		frontLeft.updateCoord();
		SmartDashboard.putNumber("Target Heading", _targetAngle);
		SmartDashboard.putNumber("turnErr", Util.boundAngleNeg180to180Degrees(Util.boundAngle0to360Degrees(intake.getCurrentAngle())-_targetAngle)); // add bounding to [-180,180]
		if(xInput == 0 && yInput == 0 && rotateInput == 0){
			frontLeft.stopDriveMotor();
			frontRight.stopDriveMotor();
			rearLeft.stopDriveMotor();
			rearRight.stopDriveMotor();
		}
		
		else{
			double A = xInput - rotateInput * (Constants.WHEELBASE_LENGTH / R);
	        double B = xInput + rotateInput * (Constants.WHEELBASE_LENGTH / R);
	        double C = yInput - rotateInput * (Constants.WHEELBASE_WIDTH / R);
	        double D = yInput + rotateInput * (Constants.WHEELBASE_WIDTH / R);
	        
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
	public void resetCoord(){
		frontLeft.resetCoord();
	}
	public boolean headingOnTarget(){
		if(Math.abs(_targetAngle - intake.getCurrentAngle()) < allowableError)
			onTarget--;
		else
			onTarget = onTargetThresh;
		return onTarget <= 0;
	}
}