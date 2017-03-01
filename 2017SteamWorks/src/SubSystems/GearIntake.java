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
		INTAKE_EXTENDED, INTAKING, INTAKE_RETRACTED, GEAR_DETECT,
		GEAR_DETECTED, CLIMB_DETECT, CLIMB_DETECTED, SCORE_GEAR_1,SCORE_GEAR_2,INTAKE_TUCK 
	}
	private State state = State.INTAKE_TUCK;
	
	public GearIntake(int canPort, int solPort){
		gear = new CANTalon(canPort);
		gear.setCurrentLimit(100);
		gear.setVoltageRampRate(36);
		gear.enableBrakeMode(true);
		shaft = new Solenoid(22,solPort);
	}
	public void update(){
		switch(state){
		case INTAKE_RETRACTED:
			stop();
			shaft.set(true);
			SmartDashboard.putString("GEAR_INTAKE", "RETRACTED");
			break;
		case INTAKING:
			gear.setCurrentLimit(100);
			gear.configMaxOutputVoltage(12f);
			forward();
			shaft.set(false);
			state = State.GEAR_DETECT;
			SmartDashboard.putString("GEAR_INTAKE", "INTAKING");
			break;
		case INTAKE_EXTENDED:
			stop();
			shaft.set(false);
			SmartDashboard.putString("GEAR_INTAKE", "EXTENDED");
			break;
		case GEAR_DETECT:
			if(gear.getOutputCurrent() > Constants.GEAR_INTAKE_CURR_DETECT){
				gear.set(-0.2);
				state = State.GEAR_DETECTED;
			}
			SmartDashboard.putString("GEAR_INTAKE", "WAITING FOR GEAR");
			break;
		case GEAR_DETECTED:
			SmartDashboard.putString("GEAR_INTAKE", "GEAR DETECTED");
			break;
		case SCORE_GEAR_1:
			gear.configMaxOutputVoltage(5f);
			gear.setCurrentLimit(15);
			timeout = System.currentTimeMillis() + solenoidTime;
			shaft.set(true);
			reverse();
//			state = State.SCORE_GEAR_2;
			break;
		case SCORE_GEAR_2:
			if(timeout < System.currentTimeMillis()){
				stop();
			}
			break;
		case INTAKE_TUCK:
			
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
		gear.set(Constants.GEAR_INTAKE_POWER);
	}
	public void reverse(){
		gear.set(-Constants.GEAR_INTAKE_POWER);
	}
	public void stop(){
		gear.set(0);
	}
}
