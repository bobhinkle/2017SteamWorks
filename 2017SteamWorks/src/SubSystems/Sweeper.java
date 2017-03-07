package SubSystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import IO.Logger;
import Utilities.Constants;
import Utilities.Ports;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Sweeper {
	private static Sweeper instance = null;
	public CANTalon sweeper;
	public CANTalon sweeper_roller;
	private boolean jammed = false;
	long timeout = 0;
	private Logger log;
	
	public Sweeper(){
		sweeper = new CANTalon(Ports.SWEEPER);
		sweeper.setVoltageRampRate(48);
		sweeper_roller = new CANTalon(Ports.SWEEPER_ROLLER);
		sweeper_roller.setVoltageRampRate(48);
		log = Logger.getInstance();
	}
	public static Sweeper getInstance(){
		if(instance == null)
			instance = new Sweeper();
		return instance;
	}
	public void forwardSweeper(){
		sweeper.set(Constants.SWEEPER_FORWARD);
	}
	public void reducedForward(){
		sweeper.set(Constants.SWEEPER_REDUCED_FORWARD);
	}
	public void reverseSweeper(){
		sweeper.set(Constants.SWEEPER_REVERSE);
	}
	public void stopSweeper(){
		sweeper.set(0);
	}
	public void forwardRoller(){
		sweeper_roller.set(Constants.SWEEPER_ROLLER_FORWARD);
	}
	public void reverseRoller(){
		sweeper_roller.set(Constants.SWEEPER_ROLLER_REVERSE);
	}
	public void stopRoller(){
		sweeper_roller.set(0);
	}
	public void SweeperDebug(){
		SmartDashboard.putNumber(" Sweeper Rotor Current ", sweeper.getOutputCurrent());
		SmartDashboard.putNumber(" Sweeper Roller Current ", sweeper_roller.getOutputCurrent());
		SmartDashboard.putNumber(" Sweeper Rotor Voltage", sweeper.getOutputVoltage());
		SmartDashboard.putNumber(" Sweeper Roller Voltage", sweeper_roller.getOutputVoltage());
	}
	public void update(){
		if(sweeper_roller.getOutputCurrent() > Constants.SWEEPER_JAM_CURRENT && !jammed){
			jammed = true;
			timeout = System.currentTimeMillis();
			stopSweeper();
			reverseRoller();
			log.writeToLog("Jammed");
		}
		if(jammed && System.currentTimeMillis() > timeout + 125){
			jammed = false;
			forwardRoller();
			reducedForward();
		}
	}
}
