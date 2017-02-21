package SubSystems;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Solenoid;

public class GearIntake {
	private CANTalon gear;
	private Solenoid shaft;
	public GearIntake(int canPort, int solPort){
		gear = new CANTalon(canPort);
		gear.setCurrentLimit(40);
		gear.setVoltageRampRate(24);
		shaft = new Solenoid(21,solPort);
	}
	public void update(){
		if(gear.getOutputCurrent() > 35){
			gear.set(0.0);
		}
	}
	public void extend(){
		shaft.set(true);
	}
	public void retract(){
		shaft.set(false);
	}
	public void forward(){
		gear.set(1);
	}
	public void reverse(){
		gear.set(-1);
	}
	public void stop(){
		gear.set(0);
	}
}
