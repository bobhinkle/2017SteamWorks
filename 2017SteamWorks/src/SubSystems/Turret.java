package SubSystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import Utilities.Constants;
import Utilities.Ports;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret {
	private static Turret instance = null;
	private CANTalon motor;
//	private int absolutePosition;
	public Turret(){
		motor = new CANTalon(Ports.TURRET);
		//motor.configPeakOutputVoltage(+6f, -6f);
		//motor.setCurrentLimit(8);
//		absolutePosition = motor.getPulseWidthPosition() & 0xFFF;
    	motor.setEncPosition(0);
    	motor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
    	motor.reverseSensor(false);
    	motor.reverseOutput(true);
    	motor.configEncoderCodesPerRev(360);
    	motor.configNominalOutputVoltage(+0f, -0f);
    	motor.configPeakOutputVoltage(+6f, -6f);
    	motor.setAllowableClosedLoopErr(0); 
    	motor.changeControlMode(TalonControlMode.Position);
    	motor.set(0);
    	//motor.setPID(1.27, 0.0, 2, 0.0, 0, 0.0, 0);
		
	}
	public static Turret getInstance(){
		if(instance == null)
			instance = new Turret();
		return instance;
	}	
	public void manualControl(double input){
		double newAngle = (motor.getSetpoint() * Constants.TURRET_CLICKS_TO_ANGLE) + (input * 3.5);
		setAngle(newAngle);		
		SmartDashboard.putNumber("TURRET_CURR", motor.getOutputCurrent());
//		motor.set(-input *0.5);
	}
	public void setAngle(double angle){
		if(angle > Constants.TURRET_MAX_ANGLE)
			angle = Constants.TURRET_MAX_ANGLE;
		if(angle < -Constants.TURRET_MAX_ANGLE)
			angle = -Constants.TURRET_MAX_ANGLE;
		motor.set(angle/Constants.TURRET_CLICKS_TO_ANGLE);
	}
	public void moveDegrees(double degree){
		double newAngle = getAngle() - degree;
		setAngle(newAngle);
	}
	public double getAngle(){
		return motor.getPosition() * Constants.TURRET_CLICKS_TO_ANGLE;
	}
	public void update(){
		SmartDashboard.putNumber("TURRET_ANGLE", motor.getPosition() * Constants.TURRET_CLICKS_TO_ANGLE);
		SmartDashboard.putNumber("TURRET_GOAL", motor.getSetpoint() * Constants.TURRET_CLICKS_TO_ANGLE);
	}
}
