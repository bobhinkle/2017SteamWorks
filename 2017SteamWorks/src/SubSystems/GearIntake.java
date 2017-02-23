package SubSystems;

import com.ctre.CANTalon;

import Utilities.Constants;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearIntake {
	private CANTalon gear;
	private Solenoid shaft;
	private long solenoidTime = 500;
	private long timeout = 0;
	public enum State{
		INTAKE_EXTENDED, INTAKING, INTAKE_RETRACTED, GEAR_DETECT, GEAR_DETECTED, CLIMB_DETECT, CLIMB_DETECTED, SCORE_GEAR_1,SCORE_GEAR_2 
	}
	private State state = State.INTAKE_RETRACTED;
	
	public GearIntake(int canPort, int solPort){
		gear = new CANTalon(canPort);
		gear.setCurrentLimit(100);
		gear.setVoltageRampRate(24);
		shaft = new Solenoid(21,solPort);
	}
	public void update(){
		switch(state){
		case INTAKE_RETRACTED:
			gear.set(0);
			shaft.set(false);
			SmartDashboard.putString("GEAR_INTAKE", "RETRACTED");
			break;
		case INTAKING:
			gear.set(Constants.GEAR_INTAKE_POWER);
			shaft.set(true);
			state = State.GEAR_DETECT;
			SmartDashboard.putString("GEAR_INTAKE", "INTAKING");
			break;
		case INTAKE_EXTENDED:
			gear.set(0);
			shaft.set(true);
			SmartDashboard.putString("GEAR_INTAKE", "INTAKE EXTENDED");
			break;
		case GEAR_DETECT:
			if(gear.getOutputCurrent() > Constants.GEAR_INTAKE_CURR_DETECT){
				gear.set(0.0);
				state = State.GEAR_DETECTED;
			}
			SmartDashboard.putString("GEAR_INTAKE", "WAITING FOR GEAR");
			break;
		case GEAR_DETECTED:
			SmartDashboard.putString("GEAR_INTAKE", "GEAR DETECTED");
			break;
		case SCORE_GEAR_1:
//			timeout = System.currentTimeMillis() + solenoidTime;
//			shaft.set(true);
			gear.set(-Constants.GEAR_INTAKE_POWER);
			break;
		case SCORE_GEAR_2:
			if(timeout < System.currentTimeMillis()){
				gear.set(-Constants.GEAR_INTAKE_POWER);
			}
			break;
		default:
			
			break;
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
		gear.set(1);
	}
	public void reverse(){
		gear.set(-1);
	}
	public void stop(){
		state = State.INTAKE_EXTENDED;
	}
}
