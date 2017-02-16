package SubSystems;

import com.ctre.CANTalon;

import Utilities.Ports;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret {
	private static Turret instance = null;
	private CANTalon motor;
	
	public Turret(){
		motor = new CANTalon(Ports.TURRET);
		motor.configPeakOutputVoltage(+6f, -6f);
		motor.setCurrentLimit(8);
	}
	public static Turret getInstance(){
		if(instance == null)
			instance = new Turret();
		return instance;
	}	
	public void moveMotor(double input){
		SmartDashboard.putNumber("TURRET_C", motor.getOutputCurrent());
		motor.set(-input *0.5);
	}
}
