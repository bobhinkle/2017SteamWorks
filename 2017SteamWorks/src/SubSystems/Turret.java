package SubSystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import Utilities.Constants;
import Utilities.Ports;
import Utilities.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret {
	private static Turret instance = null;
	private CANTalon motor;
	private double lockedAngle = 0.0;
	private double lockedTurretAngle = 0.0;
//	private int absolutePosition;
	public Turret(){
		motor = new CANTalon(Ports.TURRET);
    	motor.setEncPosition(0);
    	motor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
    	motor.reverseSensor(false);
    	motor.reverseOutput(true);
    	motor.configEncoderCodesPerRev(360);
    	motor.configNominalOutputVoltage(+0f, -0f);
    	motor.configPeakOutputVoltage(+5f, -5f);
    	motor.setAllowableClosedLoopErr(0); 
    	motor.changeControlMode(TalonControlMode.Position);
    	motor.set(0);
    	motor.setPID(1.75, 0, 25, 0.0, 0, 0.0, 0);			//practice bot pid tuning
		motor.enableBrakeMode(true);
		motor.setNominalClosedLoopVoltage(12);
	}
	public enum State{
		Off, VisionTracking, CalculatedTracking, Manual, GyroComp
	}
	public State currentState = State.Manual;
	public static Turret getInstance(){
		if(instance == null)
			instance = new Turret();
		return instance;
	}	
	public void setState(State newState){
		currentState = newState;
	}
	public State getCurrentState(){
		return currentState;
	}
	public void lockAngle(double newAngle){
		lockedAngle = newAngle;
		lockedTurretAngle = getAngle();
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
	public double getGoal(){
		return motor.getSetpoint() * Constants.TURRET_CLICKS_TO_ANGLE;
	}
	public void update(double heading){
		switch(currentState){
		case GyroComp:
			setAngle(lockedTurretAngle + (lockedAngle - heading));
			break;
		}
//		SmartDashboard.putNumber("Turret_LockedAngle", lockedTurretAngle);
//		SmartDashboard.putNumber("TurretLockedHeading", lockedAngle);
		SmartDashboard.putNumber("TURRET_ANGLE", getAngle());
		SmartDashboard.putNumber("TURRET_GOAL", getGoal());
		SmartDashboard.putNumber("TURRET_ERROR", getGoal()-getAngle());
	}
}
