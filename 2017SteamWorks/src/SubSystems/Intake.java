package SubSystems;

import com.ctre.CANTalon;

import Sensors.PigeonImu;
import Sensors.PigeonImu.PigeonState;
import Utilities.Ports;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {

	private static Intake instance = null;
	public CANTalon intakeRight;
	private double currentAngle = 0.0;
    public PigeonImu _pidgey;
    boolean angleIsGood = false;
    double currentAngularRate = 0.0;
	public Intake(){
		intakeRight = new CANTalon(Ports.INTAKE_MOTOR_R);
		_pidgey = new PigeonImu(intakeRight);
	}
	public static Intake getInstance(){
		if(instance == null)
			instance = new Intake();
		return instance;
	}
	public void pigeonUpdate(){
		PigeonImu.GeneralStatus genStatus = new PigeonImu.GeneralStatus();
		PigeonImu.FusionStatus fusionStatus = new PigeonImu.FusionStatus();
		double [] xyz_dps = new double [3];
		currentAngle = -_pidgey.GetFusedHeading(fusionStatus);
		_pidgey.GetGeneralStatus(genStatus);
		_pidgey.GetRawGyro(xyz_dps);
		angleIsGood = (_pidgey.GetState() == PigeonState.Ready) ? true : false;
		currentAngularRate = -xyz_dps[2];
		SmartDashboard.putNumber("Pigeon_CA", currentAngle);
		SmartDashboard.putNumber("PigeonRate", currentAngularRate);
		SmartDashboard.putBoolean("PigeonGood", angleIsGood);
	}
	public double getCurrentAngle(){
		return currentAngle;
	}
	public double getCurrentAngularRate(){
		return currentAngularRate;
	}
}
