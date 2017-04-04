package SubSystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import Utilities.Constants;
import Utilities.Ports;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; //added

public class Turret {
	private static Turret instance = null;
	private CANTalon motor;
	private double lockedAngle = 0.0;
	private double lockedTurretAngle = 0.0;
	private int onTargetCheck = 0;
	private double angleOffset = 0;
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
    	motor.setPID(Constants.TURRET_DEFAULT_P, 0.0, Constants.TURRET_DEFAULT_D, 0.0, 0, 0.0, 0);	//practice bot pid tuning
    	motor.setPID(Constants.TURRET_SMALL_P, 0.00, Constants.TURRET_SMALL_P, 0.0, 0, 0.0, 1);
    	motor.setProfile(0);
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
	
	public void lockAngle(double newAngle,double turretAngle){
		lockedAngle = newAngle;
		lockedTurretAngle = turretAngle;
	}
	
	public void manualControl(double input){
		double newAngle = (motor.getSetpoint() * Constants.TURRET_CLICKS_TO_ANGLE) + (input * 2);
		setAngle(newAngle);		
		onTargetCheck = Constants.TURRET_ONTARGET_THRESH;
	}
	public void setAngle(double angle){
		if(angle > Constants.TURRET_MAX_ANGLE)
			angle = Constants.TURRET_MAX_ANGLE;
		if(angle < -Constants.TURRET_MAX_ANGLE)
			angle = -Constants.TURRET_MAX_ANGLE;
		motor.set(angle/Constants.TURRET_CLICKS_TO_ANGLE);
		onTargetCheck = Constants.TURRET_ONTARGET_THRESH;
	}
	public void moveDegrees(double degree){
		double newAngle = getAngle() - degree;
		setAngle(newAngle);
	}
	public double getAngle(){
		return (motor.getPosition() * Constants.TURRET_CLICKS_TO_ANGLE);
	}
	public double getGoal(){
		return (motor.getSetpoint() * Constants.TURRET_CLICKS_TO_ANGLE);
	}
	public void update(double heading){
		if(Math.abs(getError()) < Constants.TURRET_SMALL_PID_THRESH){
			motor.setProfile(1);
		}else{
			motor.setProfile(0);
		}
		switch(currentState){
		case GyroComp:
			setAngle(lockedTurretAngle + (lockedAngle - heading));
			break;
		default:
			break;
		}
		
		if(motor.getOutputCurrent() > 30){
			motor.setSetpoint(motor.getPosition());
		}
//		SmartDashboard.putNumber("Turret_LockedAngle", lockedTurretAngle);
//		SmartDashboard.putNumber("TurretLockedHeading", lockedAngle);
		
		SmartDashboard.putNumber("TURRET_ANGLE", getAngle());
		SmartDashboard.putNumber("TURRET_GOAL", getGoal());
		SmartDashboard.putNumber("TURRET_ERROR", getError());
//		Util.sdVerboseClosedLoop("Turret", "Angle", getAngle(), getGoal(),motor.getOutputCurrent()); // *** NEW! ***
		SmartDashboard.putNumber("TURRET_CURR", motor.getOutputCurrent());
		SmartDashboard.putNumber("TURRET_VOLTAGE", motor.getOutputVoltage());
	}
	public double getError(){
		return (getGoal() - getAngle());
	}
	public boolean onTarget(){
		if(Math.abs(getError()) < 2.5){
			onTargetCheck--;
		}else{
			onTargetCheck = Constants.TURRET_ONTARGET_THRESH;
		}
		return onTargetCheck <= 0;
	}
	public void resetAngle(double a){
		motor.setEncPosition((int)a*Constants.TURRET_TICKS_PER_90);
		motor.set(a/Constants.TURRET_CLICKS_TO_ANGLE);
	}
}
