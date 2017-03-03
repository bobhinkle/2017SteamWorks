package SubSystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import Utilities.Constants;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearIntake {
	private CANTalon gear;
	private Solenoid gearCylinder;
	private long solenoidTime = 500;
	private long timeout = 0;
	private autoDeploy auto;
	public enum State{
		INTAKE_EXTENDED, INTAKING, INTAKE_RETRACTED, GEAR_DETECT,
		GEAR_DETECTED, CLIMB_DETECT, CLIMB_DETECTED, SCORE_GEAR_1,SCORE_GEAR_2,INTAKE_TUCK,INTAKE_EXTENDED_OFF,INTAKE_RETRACTED_WITH_GEAR,
		GEAR_LOST, GEAR_AUTO
	}
	private State state = State.INTAKE_TUCK;
	
	public GearIntake(int canPort, int solPort){
		gear = new CANTalon(canPort);
		//gear.setCurrentLimit(40);
		//gear.setVoltageRampRate(36);
		gear.enableBrakeMode(true);
		gear.changeControlMode(TalonControlMode.Current);
		gear.setPID(0.04, 0.00, 0, 0.08, 0, 0.0, 0);
		gear.reverseOutput(true);
		gear.setCloseLoopRampRate(36);
		gearCylinder = new Solenoid(22,solPort);
	}
	public void update(){
//		String gearIntakeStatus = null;
		switch(state){
			case INTAKE_EXTENDED_OFF:
				stop();
				gearCylinder.set(false);
				SmartDashboard.putString(" Gear Intake Status ", "EXTENDED OFF");
				break;
	
			case INTAKE_RETRACTED:
				stop();
				gearCylinder.set(true);
				
				SmartDashboard.putString(" Gear Intake Status ", "RETRACTED");
				break;
			case INTAKE_RETRACTED_WITH_GEAR:
				gearCylinder.set(true);
				if(gear.getOutputCurrent() < Constants.GEAR_PRESENT){
					state = State.GEAR_LOST;
				}
				SmartDashboard.putString(" Gear Intake Status ", "RETRACTED WITH GEAR");
				break;
			case INTAKING:
				forward();
				gearCylinder.set(false);
				state = State.GEAR_DETECT;
				SmartDashboard.putString(" Gear Intake Status ", "INTAKING");
				break;
	
			case INTAKE_EXTENDED:
				stop();
				gearCylinder.set(false);
				SmartDashboard.putString(" Gear Intake Status ", "EXTENDED");
				break;
	
			case GEAR_DETECT:
				if(gear.getOutputCurrent() > Constants.GEAR_INTAKE_CURR_DETECT){
					gear.changeControlMode(TalonControlMode.PercentVbus);
					gear.set(Constants.GEAR_INTAKE_HOLDING_POWER);
					state = State.GEAR_DETECTED;
				}
				SmartDashboard.putString(" Gear Intake Status ", "WAITING FOR GEAR");
				break;
	
			case GEAR_DETECTED:
				if(gear.getOutputCurrent() < Constants.GEAR_PRESENT){
					state = State.GEAR_LOST;
				}
				SmartDashboard.putString(" Gear Intake Status ", "GEAR DETECTED");
				break;
			case GEAR_LOST:
				SmartDashboard.putString(" Gear Intake Status ", "GEAR LOST");
				break;
			case SCORE_GEAR_1:
				timeout = System.currentTimeMillis() + solenoidTime;
				gearCylinder.set(false);
				reverse();
				state = State.SCORE_GEAR_2;
				break;
	
			case SCORE_GEAR_2:
				if(gear.getOutputCurrent() > Constants.GEAR_INTAKE_REVERSE_CURR_DETECT){
					//gear.set(Constants.GEAR_INTAKE_REVERSE_POWER);
				}
				break;
	
			case INTAKE_TUCK: break;
			default: 
				
				break;
		}
		//SmartDashboard.putString(" Gear Intake Status ", gearIntakeStatus);
		//SmartDashboard.putNumber("Gear Intake Current" , gear.getOutputCurrent());
		SmartDashboard.putNumber("Gear Intake Error" ,gear.getSetpoint()-gear.getOutputCurrent());
		SmartDashboard.putNumber("Gear Intake Voltage" ,gear.getOutputVoltage());
	}
	public void extend(){
		if(state == State.GEAR_LOST || state == State.INTAKING || state == State.GEAR_DETECT)
			state = State.INTAKING;
		else
			state = State.INTAKE_EXTENDED;
	}
	public void retract(){
		if(state != State.GEAR_DETECTED && state != State.INTAKE_RETRACTED_WITH_GEAR && state != State.GEAR_LOST){
			state = State.INTAKE_RETRACTED;
		}else{
			state  = State.INTAKE_RETRACTED_WITH_GEAR;
		}
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
		gear.changeControlMode(TalonControlMode.Current);
		gear.setProfile(0);
		gear.set(Constants.GEAR_INTAKE_POWER);
	}
	public void reverse(){
		gear.changeControlMode(TalonControlMode.Current);
		gear.setProfile(0);
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
	public void autoDep(){
		auto = new autoDeploy();
		auto.start();
	}
	public class autoDeploy extends Thread{
		public void run(){
			state = GearIntake.State.GEAR_AUTO;
			gear.changeControlMode(TalonControlMode.Current);
			gear.setProfile(0);
			gear.set(10);
			Timer.delay(1);
			gearCylinder.set(true);
			Timer.delay(2);
		}
	}
}
