package SubSystems;

import com.ctre.CANTalon;

import Utilities.Ports;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Sweeper {
	private static Sweeper instance = null;
	public CANTalon sweeper;
	public CANTalon sweeper_roller;
	
	public Sweeper(){
		sweeper = new CANTalon(Ports.SWEEPER);
		sweeper.setVoltageRampRate(36);
		sweeper_roller = new CANTalon(Ports.SWEEPER_ROLLER);
		sweeper_roller.setVoltageRampRate(36);
	}
	public static Sweeper getInstance(){
		if(instance == null)
			instance = new Sweeper();
		return instance;
	}
	public void forwardSweeper(){
		sweeper.set(-1);
	}
	public void reverseSweeper(){
		sweeper.set(1);
	}
	public void stopSweeper(){
		sweeper.set(0);
	}
	public void forwardRoller(){
		sweeper_roller.set(1);
	}
	public void reverseRoller(){
		sweeper_roller.set(-1);
	}
	public void stopRoller(){
		sweeper_roller.set(0);
	}
	public void SweeperDebug(){
		SmartDashboard.putNumber("Sweeper_Curr", sweeper.getOutputCurrent());
		SmartDashboard.putNumber("SweepRoller", sweeper_roller.getOutputCurrent());
	}
}