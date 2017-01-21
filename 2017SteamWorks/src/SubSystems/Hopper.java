package SubSystems;

import com.ctre.CANTalon;

import Utilities.Ports;
import edu.wpi.first.wpilibj.Solenoid;

public class Hopper {

	public class Intake{
		private CANTalon motor;
		private double maxPower = 0.5;
		private Solenoid intake_arm;
		public Intake(){
			motor = new CANTalon(Ports.INTAKE_MOTOR);
			intake_arm = new Solenoid(Ports.INTAKE_ARM);
		}
		
		public void forward(){
			motor.set(maxPower);
		}
		public void reverse(){
			motor.set(-maxPower);
		}
		public void stop(){
			motor.set(0);
		}
		public void extend(){
			intake_arm.set(true);
		}
		public void retract(){
			intake_arm.set(false);
		}
	}
	
	Intake intake = new Intake();
}
