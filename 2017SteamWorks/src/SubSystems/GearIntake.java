package SubSystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import IO.Logger;
import Utilities.Constants;
import Utilities.Ports;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearIntake {
	private CANTalon gear;
	private Solenoid gearCylinder;
	private long solenoidTime = 500;
	private long timeout = 0;
	private autoDeploy auto;
	private autoPickup pickup;
	private Logger logger;
	private int cyclesPastThresh = 1;
	private int cyclesThresh = 1;
	private boolean isScoring = false;
	public boolean needsToVibrate = false;
	public enum State{
		INTAKE_EXTENDED, INTAKING, INTAKE_RETRACTED, GEAR_DETECT,
		GEAR_DETECTED, CLIMB_DETECT, CLIMB_DETECTED, SCORE_GEAR_1,SCORE_GEAR_2,INTAKE_TUCK,INTAKE_EXTENDED_OFF,INTAKE_RETRACTED_WITH_GEAR,
		GEAR_LOST_EXTENDED, GEAR_AUTO, GEAR_LOST_RETRACTED, INTAKE_RETRACTED_GEAR_DETECT, INTAKE_RETRACTED_GEAR_DETECTED
	}
	private State state = State.INTAKE_TUCK;
	
	public GearIntake(int canPort, int solPort){
		gear = new CANTalon(canPort);
		//gear.setCurrentLimit(40);
		//gear.setVoltageRampRate(36);
		gear.enableBrakeMode(false);
		gear.changeControlMode(TalonControlMode.Current);
		gear.setPID(0.06, 0.00, 0, 0.08, 0, 0.0, 0);
		gear.reverseOutput(true);
		gear.setCloseLoopRampRate(12);
		gearCylinder = new Solenoid(20,solPort);
		logger = Logger.getInstance();
	}
	public void update(){
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
					state = State.GEAR_LOST_RETRACTED;
				}
				SmartDashboard.putString(" Gear Intake Status ", "RETRACTED WITH GEAR");
				break;
			case INTAKING:
				forward();
				gearCylinder.set(false);
				cyclesPastThresh = cyclesThresh;
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
						state = State.GEAR_DETECTED;
						System.out.println("Gear Detected");
				}
				SmartDashboard.putString(" Gear Intake Status ", "WAITING FOR GEAR");
				break;
	
			case GEAR_DETECTED:
				gear.changeControlMode(TalonControlMode.PercentVbus);
				gear.set(Constants.GEAR_INTAKE_HOLDING_POWER);
				if(gear.getOutputCurrent() < Constants.GEAR_PRESENT){
					state = State.GEAR_LOST_EXTENDED;
				}
				SmartDashboard.putString(" Gear Intake Status ", "GEAR DETECTED");
				needsToVibrate = false;
				break;
			case GEAR_LOST_EXTENDED:
//				state = State.INTAKING;
				SmartDashboard.putString(" Gear Intake Status ", "GEAR LOST");
				break;
			case GEAR_LOST_RETRACTED:
				SmartDashboard.putString(" Gear Intake Status ", "GEAR LOST");
				break;
			case SCORE_GEAR_1:
				SmartDashboard.putString(" Gear Intake Status ", "SCORE GEAR 1");
				if(!isScoring){
					autoDep();
				}
				break;
	
			case SCORE_GEAR_2:
				if(gear.getOutputCurrent() > Constants.GEAR_INTAKE_REVERSE_CURR_DETECT){
					//gear.set(Constants.GEAR_INTAKE_REVERSE_POWER);
				}
				break;
	
			case INTAKE_TUCK: break;
			case INTAKE_RETRACTED_GEAR_DETECT:
				forward();
				
				break;
			case INTAKE_RETRACTED_GEAR_DETECTED:
				if(gear.getOutputCurrent() > Constants.GEAR_INTAKE_CURR_DETECT){
					gear.changeControlMode(TalonControlMode.PercentVbus);
					gear.set(Constants.GEAR_INTAKE_HOLDING_POWER);
					state = State.INTAKE_RETRACTED_WITH_GEAR;
				}
				break;
			default: 
				
				break;
		}
		//SmartDashboard.putString(" Gear Intake Status ", gearIntakeStatus);
		//SmartDashboard.putNumber("Gear Intake Current" , gear.getOutputCurrent());
		SmartDashboard.putNumber("Gear Intake Error" ,gear.getSetpoint()-gear.getOutputCurrent());
		SmartDashboard.putNumber("Gear Intake Voltage" ,gear.getOutputVoltage());
	}
	public void firePiston(){
		gearCylinder.set(true);
	}
	public void extend(){
		if(state == State.GEAR_LOST_EXTENDED || state == State.GEAR_LOST_RETRACTED || state == State.INTAKING || state == State.GEAR_DETECT || state == State.INTAKE_EXTENDED_OFF)
			state = State.INTAKING;
		else
			state = State.INTAKE_EXTENDED;
	}
	public void retract(){
		if(state != State.GEAR_DETECTED && state != State.INTAKE_RETRACTED_WITH_GEAR && state != State.GEAR_LOST_EXTENDED && state != State.GEAR_LOST_RETRACTED){
			state = State.INTAKE_RETRACTED;
		}else{
			state  = State.INTAKE_RETRACTED_WITH_GEAR;
		}
		gear.changeControlMode(TalonControlMode.PercentVbus);
		gear.set(Constants.GEAR_INTAKE_HOLDING_POWER);
	}
	public void grabGear(){
		if(state != State.INTAKE_RETRACTED){
			state = State.INTAKING;
		}else{
			gearCylinder.set(false);
			state = State.INTAKING;
		}
		needsToVibrate = true;
	}
	public void scoreGear(){
		if(state != State.SCORE_GEAR_1)
			state = State.SCORE_GEAR_1;
	}
	public void forward(){
		gear.changeControlMode(TalonControlMode.PercentVbus);
		
		
		gear.set(-Constants.GEAR_INTAKE_POWER);
		//gear.setProfile(0);
	}
	public void reverse(){
		gear.set(0);
		gear.changeControlMode(TalonControlMode.Current);
//		gear.setProfile(0);
		gear.set(-Constants.GEAR_INTAKE_POWER_REVERSE);
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
	public void initiateAutoPickup(){
		pickup = new autoPickup();
		pickup.start();
	}
	public class autoDeploy extends Thread{
		public void run(){
			isScoring = true;
			gear.set(0);
			Timer.delay(0.1);
			gear.changeControlMode(TalonControlMode.Current);
			//gear.setProfile(0);
			gear.set(-12);
			Timer.delay(0.25);
			gearCylinder.set(false);
			Timer.delay(0.75);
			state = GearIntake.State.INTAKE_EXTENDED_OFF;
			isScoring = false;
		}
	}
	public class autoPickup extends Thread{
		public void run(){
			state = GearIntake.State.INTAKING;
			Timer.delay(0.1);
			forward();
			state = GearIntake.State.GEAR_DETECT;
			Timer.delay(0.1);
			forward();
			state = GearIntake.State.GEAR_DETECT;
			long timeout = System.currentTimeMillis() + 1000;
			while(System.currentTimeMillis() < timeout && state != GearIntake.State.GEAR_DETECTED){
				Timer.delay(0.005);
			}
			forward();
			state = GearIntake.State.GEAR_DETECT;
			timeout = System.currentTimeMillis() + 500;
			while(System.currentTimeMillis() < timeout && state != GearIntake.State.GEAR_DETECTED){
				Timer.delay(0.005);
			}
			Timer.delay(0.1);
			gear.changeControlMode(TalonControlMode.PercentVbus);
			gear.set(Constants.GEAR_INTAKE_HOLDING_POWER);
			state = GearIntake.State.INTAKE_RETRACTED_WITH_GEAR;
			System.out.println("Gear intake Retracted");
		}
	}
	public void gearExtender(){
		gearCylinder.set(true);
	}
}
