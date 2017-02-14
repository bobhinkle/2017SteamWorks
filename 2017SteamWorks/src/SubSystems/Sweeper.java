package SubSystems;

import com.ctre.CANTalon;

import Utilities.Ports;

public class Sweeper {

	public CANTalon sweeper;
	public CANTalon sweeper_roller;
	
	public Sweeper(){
		sweeper = new CANTalon(Ports.SWEEPER);
		sweeper_roller = new CANTalon(Ports.SWEEPER_ROLLER);
	}
	
	public void forward(){
		sweeper.set(1);
		sweeper_roller.set(1);
	}
	public void reverse(){
		sweeper.set(-1);
		sweeper_roller.set(-1);
	}
	public void stop(){
		sweeper.set(0);
		sweeper_roller.set(0);
	}
}
