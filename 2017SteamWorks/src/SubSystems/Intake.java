package SubSystems;

import com.ctre.CANTalon;
import com.ctre.PigeonImu;
import com.ctre.PigeonImu.PigeonState;

import Utilities.Ports;
import Utilities.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {

	private static Intake instance = null;
	public CANTalon intakeLeft;
	public CANTalon intakeRight;
	private double currentAngle = 0.0;
    public PigeonImu _pidgey;
    boolean angleIsGood = false;
    double currentAngularRate = 0.0;
	public Intake(){
		intakeLeft = new CANTalon(Ports.INTAKE_MOTOR_L);
		intakeLeft.configNominalOutputVoltage(12.0f, -12.0f);
		intakeLeft.setVoltageRampRate(24);
		intakeRight = new CANTalon(Ports.INTAKE_MOTOR_R);
		intakeRight.configNominalOutputVoltage(12.0f, -12.0f);
		intakeRight.setVoltageRampRate(24);
		try{
			_pidgey = new PigeonImu(intakeLeft);
		}catch(Exception e){
			System.out.println(e);
		}
	}
	public static Intake getInstance(){
		if(instance == null)
			instance = new Intake();
		return instance;
	}
	public void pigeonUpdate(){
		try{
			PigeonImu.GeneralStatus genStatus = new PigeonImu.GeneralStatus();
			PigeonImu.FusionStatus fusionStatus = new PigeonImu.FusionStatus();
			double [] xyz_dps = new double [3];
			currentAngle = -_pidgey.GetFusedHeading(fusionStatus);
			_pidgey.GetGeneralStatus(genStatus);
			_pidgey.GetRawGyro(xyz_dps);
			angleIsGood = (_pidgey.GetState() == PigeonState.Ready) ? true : false;
			currentAngularRate = -xyz_dps[2];
			SmartDashboard.putNumber("Pigeon_CA", currentAngle);
			SmartDashboard.putNumber(" Pigeon Rate ", currentAngularRate);
			SmartDashboard.putBoolean("PigeonGood", angleIsGood);
		}catch(Exception e){
			System.out.println(e);
		}
	}
	public double getCurrentAngle(){
		return currentAngle;
	}
	public double getCurrentAngularRate(){
		return currentAngularRate;
	}
	public void intakeForward(){
		intakeLeft.set(-1.0); 
		intakeRight.set(1.0);
	}
	public void intakeReverse(){
		intakeLeft.set(1.0);
		intakeRight.set(-1.0);
	}
	public void intakeStop(){
		intakeLeft.set(0);
		intakeRight.set(0);
	}
	public void debugValues(){
		SmartDashboard.putNumber("INTAKE_L_C", intakeLeft.getOutputCurrent());
		SmartDashboard.putNumber("INTAKE_R_C", intakeRight.getOutputCurrent());
	}
	public void update(){
		pigeonUpdate();
		debugValues();
	}
}
