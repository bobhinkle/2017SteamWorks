package SubSystems;
import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import Sensors.PigeonImu;
import Sensors.PigeonImu.PigeonState;
import Utilities.Constants;
import Utilities.Ports;
import Utilities.Util;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Swerve{
	private static Swerve instance = null;
	
	public SwerveDriveModule frontLeft;
	private SwerveDriveModule frontRight;
	private SwerveDriveModule rearLeft;
	private SwerveDriveModule rearRight;
	private static double R = 39.522;
	private double rotationScaleFactor = .225;
	private double rotationScaleFactorFast = 0.35;
	private double xInput,yInput,rotateInput = 0;
	
	//Pigeon IMU Data
	public PigeonImu _pidgey;
	public CANTalon _spareTalon;
	double currentAngle = 0.0;
	boolean angleIsGood = false;
	double currentAngularRate = 0.0;
	boolean tracking = false;
	//Swerve Turning Gains
	double kPgain = 0.005; /* percent throttle per degree of error */
	double kDgain = 0.004; /* percent throttle per angular velocity dps */
	double kPgainSmall = 0.005;
	double kDgainSmall = 0.004;
	double kMaxCorrectionRatioSmall = 0.14;
	double kMaxCorrectionRatio = 0.12; /* cap corrective turning throttle to 30 percent of forward throttle */
	
	double _targetAngle = 0.0;
	double rotationCorrection;
	int _printLoops = 0;
	
	public void setHeading(double goal){
		_targetAngle = continousAngle(goal,currentAngle);		
	}
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
	enum HeadingController{
		Off, On, Reset
	}
	public HeadingController headingController = HeadingController.Off;
	
	public Swerve(){
		frontLeft  = new SwerveDriveModule(Ports.FRONT_LEFT_ROTATION,Ports.FRONT_LEFT_DRIVE,2);
		frontRight = new SwerveDriveModule(Ports.FRONT_RIGHT_ROTATION,Ports.FRONT_RIGHT_DRIVE,1);
		rearLeft   = new SwerveDriveModule(Ports.REAR_LEFT_ROTATION,Ports.REAR_LEFT_DRIVE,3);
		rearRight  = new SwerveDriveModule(Ports.REAR_RIGHT_ROTATION,Ports.REAR_RIGHT_DRIVE,4);
		_spareTalon = frontLeft.driveMotor;
		_pidgey = new PigeonImu(_spareTalon);
	}
	public static Swerve getInstance()
    {
        if( instance == null )
            instance = new Swerve();
        return instance;
    }
	public void pigeonUpdate(){
		PigeonImu.GeneralStatus genStatus = new PigeonImu.GeneralStatus();
		PigeonImu.FusionStatus fusionStatus = new PigeonImu.FusionStatus();
		double [] xyz_dps = new double [3];
		currentAngle = -_pidgey.GetFusedHeading(fusionStatus);
		_pidgey.GetGeneralStatus(genStatus);
		_pidgey.GetRawGyro(xyz_dps);
		angleIsGood = (_pidgey.GetState() == PigeonState.Ready) ? true : false;
		currentAngularRate = -xyz_dps[2];
		SmartDashboard.putNumber("Pigeon_CA", currentAngle);
		SmartDashboard.putNumber("PigeonRate", currentAngularRate);
		SmartDashboard.putBoolean("PigeonGood", angleIsGood);
	}
	public void swerveTrack(){
		double adjust = currentAngle - Vision.getAngle();
    	setHeading(adjust);
    	tracking = true;
	}
	public void sendInput(double x, double y, double rotateX,double rotateY,boolean halfPower,boolean robotCentric,boolean moonMenuever){
		tracking = false;
		SmartDashboard.putNumber("X Stick", rotateX);
		SmartDashboard.putNumber("Y Stick", rotateY);
		if(moonMenuever){
			headingController = HeadingController.Off;
			yInput = 0.0;
			xInput = -(rotateX * 0.2) * ((Math.abs(y)*2)+1) ;
			rotateInput = rotateX * 0.2;
			SmartDashboard.putNumber("xMoon", xInput);
			SmartDashboard.putNumber("rMoon", rotateInput);
		}
		else{
			if((rotateX > 0.1 || rotateX < -0.1)){
				headingController = HeadingController.Off;
				rotateInput = rotateX;
			}else if(rotateX > -0.1 && rotateX < 0.1 && headingController == HeadingController.Off){
				headingController = HeadingController.On;
				_targetAngle = currentAngle;
			}else{
				rotateInput = 0.0;
			}
			double angle = currentAngle/180.0*Math.PI;
			if (headingController == HeadingController.On) {
				double angleDiff = Math.abs(_targetAngle - currentAngle);
				if(angleDiff < 5){
					rotationCorrection = (_targetAngle - currentAngle) * kPgainSmall - (currentAngularRate) * kDgainSmall;
					rotationCorrection = Cap(rotationCorrection, kMaxCorrectionRatioSmall);
				}else{
					rotationCorrection = (_targetAngle - currentAngle) * kPgain - (currentAngularRate) * kDgain;
					rotationCorrection = Cap(rotationCorrection, kMaxCorrectionRatio);
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
		
		update();
	}
	
	public class SwerveDriveModule{
		private CANTalon rotationMotor;
		public CANTalon driveMotor;
		private int moduleID;
		private Encoder encoder;
		private int absolutePosition;
		public void debugValues(){
			//Note #3
			SmartDashboard.putNumber("ROT_" + Integer.toString(moduleID), Util.boundAngle0to360Degrees(getCurrentAngle()));
			SmartDashboard.putNumber("DRV_" + Integer.toString(moduleID), driveMotor.get());
		}
		public SwerveDriveModule(int rotationMotorPort, int driveMotorPort,int moduleNum){
			rotationMotor = new CANTalon(rotationMotorPort);
			driveMotor = new CANTalon(driveMotorPort);
			loadProperties();
			moduleID = moduleNum;           
		}
		
		public void setGoal(double goalAngle)
	    {
			rotationMotor.set(continousAngle(goalAngle,rotationMotor.get()));
	    }
		public double wheelError(){
			return Math.abs(rotationMotor.getSetpoint() - rotationMotor.get());
		}
	    public final void loadProperties()
	    {
	    	absolutePosition = rotationMotor.getPulseWidthPosition() & 0xFFF;
	    	rotationMotor.setEncPosition(absolutePosition);
	    	rotationMotor.setFeedbackDevice(FeedbackDevice.AnalogEncoder);
	    	rotationMotor.reverseSensor(true);
	    	rotationMotor.reverseOutput(false);
	    	rotationMotor.configPotentiometerTurns(360);
	    	rotationMotor.configNominalOutputVoltage(+0f, -0f);
	    	rotationMotor.configPeakOutputVoltage(+12f, -12f);
	    	rotationMotor.setAllowableClosedLoopErr(0); 
	    	rotationMotor.changeControlMode(TalonControlMode.Position);
//	    	rotationMotor.setPID(0.1, 0.0, 0.4, 0.050, 0, 0.0, 0);
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
		pigeonUpdate();
		SmartDashboard.putNumber("Target Heading", _targetAngle);
		
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
			
			frontLeft.setDriveSpeed(-frontLeftWheelSpeed);
			frontRight.setDriveSpeed(frontRightWheelSpeed);
			rearLeft.setDriveSpeed(-rearLeftWheelSpeed);
			rearRight.setDriveSpeed(rearRightWheelSpeed);
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
	
	void UpdatGains() {
		kPgain = _spareTalon.getP();
		kDgain = _spareTalon.getD();
		kMaxCorrectionRatio = _spareTalon.getF();
	}

	
}