package SubSystems;

import com.ctre.CANTalon;

import Utilities.Constants;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearIntake {
	private CANTalon gear;
	private Solenoid shaft;
	private long solenoidTime = 500;
	@SuppressWarnings("unused")
	private long timeout = 0;
	public enum State{
		INTAKE_EXTENDED, INTAKING, INTAKE_RETRACTED, GEAR_DETECT,
		GEAR_DETECTED, CLIMB_DETECT, CLIMB_DETECTED, SCORE_GEAR_1,SCORE_GEAR_2,INTAKE_TUCK,INTAKE_EXTENDED_OFF
	}
	private State state = State.INTAKE_TUCK;
	
	public GearIntake(int canPort, int solPort){
		gear = new CANTalon(canPort);
		gear.setCurrentLimit(40);
		gear.setVoltageRampRate(36);
		gear.enableBrakeMode(true);
		shaft = new Solenoid(22,solPort);
	}
	public void update(){
		switch(state){
			case INTAKE_EXTENDED_OFF:
				stop();
				shaft.set(false);
				SmartDashboard.putString(" Gear Intake Status ","EXTENDED OFF");
				break;
	
			case INTAKE_RETRACTED:
				stop();
				shaft.set(true);
				SmartDashboard.putString(" Gear Intake Status ", "RETRACTED");
				break;
	
			case INTAKING:
				gear.setCurrentLimit(40);
				gear.configMaxOutputVoltage(12f);
				forward();
				shaft.set(false);
				state = State.GEAR_DETECT;
				SmartDashboard.putString(" Gear Intake Status ", "INTAKING");
				break;
	
			case INTAKE_EXTENDED:
				stop();
				shaft.set(false);
				SmartDashboard.putString(" Gear Intake Status ", "EXTENDED");
				break;
	
			case GEAR_DETECT:
				if(gear.getOutputCurrent() > Constants.GEAR_INTAKE_CURR_DETECT){
					gear.set(Constants.GEAR_INTAKE_FORWARD_POWER);
					state = State.GEAR_DETECTED;
				}
				SmartDashboard.putString(" Gear Intake Status ", "WAITING FOR GEAR");
				break;
	
			case GEAR_DETECTED:
				SmartDashboard.putString(" Gear Intake Status ", "GEAR DETECTED");
				break;
	
			case SCORE_GEAR_1:
				gear.configMaxOutputVoltage(12f);
				gear.setCurrentLimit(30);
				timeout = System.currentTimeMillis() + solenoidTime;
				shaft.set(false);
				reverse();
				state = State.SCORE_GEAR_2;
				break;
	
			case SCORE_GEAR_2:
				if(gear.getOutputCurrent() > Constants.GEAR_INTAKE_REVERSE_CURR_DETECT){
					gear.set(Constants.GEAR_INTAKE_REVERSE_POWER);
				}
				break;
	
			case INTAKE_TUCK: break;
			default: break;
		}
	}
	public void extend(){
		state = State.INTAKE_EXTENDED;
	}
	public void retract(){
		state  = State.INTAKE_RETRACTED;
	}
	public void grabGear(){
		if(state != State.INTAKE_RETRACTED){
			state = State.INTAKING;
		}
	}
	public void scoreGear(){
		if(state != State.SCORE_GEAR_1)
			state = State.SCORE_GEAR_1;
	}
	public void forward(){
		gear.set(Constants.GEAR_INTAKE_POWER);
	}
	public void reverse(){
		gear.set(-Constants.GEAR_INTAKE_POWER);
	}
	public void stop(){
		gear.set(0);
	}
	public void gearCurrent(){
		SmartDashboard.putNumber(" Gear Current ", gear.getOutputCurrent());
	}
	public void setState(State newState) {
		state = newState;
	}
	public State getState() {
		return state;
	}
}
