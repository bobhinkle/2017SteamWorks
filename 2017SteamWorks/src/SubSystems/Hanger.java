package SubSystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import IO.Logger;
import Utilities.Constants;
import Utilities.Ports;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hanger {
	private CANTalon climber;
	private Logger logger;
	private int cyclesOnThreshold = 5;
	private int cyclesCheck = 5;
	private static Hanger instance = null;
	public enum State{
		HANGING, HANG_WAITING, HANG_DETECTED, OFF
	}
	private State state = State.OFF;
	public Hanger(){
		climber = new CANTalon(Ports.HANGER);
		climber.enableBrakeMode(true);
	}
	public static Hanger getInstance(){
		if(instance == null){
			instance = new Hanger();
		}
		return instance;
	}
	
	public void update(){
		switch(state){
			case OFF:
				stop();
				SmartDashboard.putString(" Hanger Status ", "Off");
				break;
			case HANGING:
				climber.changeControlMode(TalonControlMode.Current);
				climber.set(-Constants.GEAR_HANG_CURRENT);
				state = State.HANG_WAITING;
				SmartDashboard.putString(" Hanger Status ", "Hanging");
				break;
			case HANG_WAITING:
				if(climber.getOutputCurrent()>Constants.GEAR_HANG_CURRENT_THRESHOLD){
					cyclesOnThreshold--;
				}else{
					cyclesOnThreshold = cyclesCheck;
				}
				if(cyclesOnThreshold <= 0){
					state = State.HANG_DETECTED;
				}
				SmartDashboard.putString(" Hanger Status ", "Hanging Waiting");
				logger.writeToLog("Climber Current: " + Double.toString(climber.getOutputCurrent()));
				break;
			case HANG_DETECTED:
				climber.changeControlMode(TalonControlMode.PercentVbus);
				climber.set(0);
				SmartDashboard.putString(" Hanger Status ", "Hang Detected");
				break;
		}
		SmartDashboard.putNumber("Hanger Current" ,climber.getOutputCurrent());
		SmartDashboard.putNumber("Hanger Voltage" ,climber.getOutputVoltage());
	}
	
	public void stop(){
		climber.set(0);
	}
	public State getState(){
		return state;
	}
	public void setState(State _state){
		state = _state;
	}
}
