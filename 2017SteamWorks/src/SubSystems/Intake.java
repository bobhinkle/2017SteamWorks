package SubSystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
import com.ctre.PigeonImu;
import com.ctre.PigeonImu.PigeonState;

import Utilities.Ports;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {

	private static Intake instance = null;
	public CANTalon intakeLeft;
	public CANTalon intakeRight;
	private double currentAngle = 0.0;
    private PigeonImu _pidgey;
    boolean angleIsGood = false;
    double currentAngularRate = 0.0;
    public enum AnglePresets{
		ZERO,NINETY,ONE_EIGHTY, TWO_SEVENTY
	}
	public Intake(){
		intakeLeft = new CANTalon(Ports.INTAKE_MOTOR_L);
		intakeLeft.changeControlMode(TalonControlMode.Current);
		intakeLeft.setPID(0.01, 0.00, 0, 0.0175, 0, 0.0, 0);
		intakeLeft.reverseOutput(true);
		intakeLeft.setCloseLoopRampRate(24);
		intakeRight = new CANTalon(Ports.INTAKE_MOTOR_R);
		intakeRight.changeControlMode(TalonControlMode.Current);
		intakeRight.setPID(0.01, 0.00, 0, 0.0175, 0, 0.0, 0);
		intakeRight.reverseOutput(false);
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
			SmartDashboard.putNumber(" Heading Angle ", currentAngle); // moved to Swerve.update()
//			SmartDashboard.putNumber(" Pigeon Rate ", currentAngularRate);
			SmartDashboard.putBoolean(" Pigeon Good ", angleIsGood);
		}catch(Exception e){
			System.out.println(e);
		}
	}
	public boolean pidgeyGood(){
		return angleIsGood;
	}
	public double getCurrentAngle(){
		return currentAngle;
		
	}
	public double getCurrentAngularRate(){
		return currentAngularRate;
	}
	public void intakeForward(){
		intakeLeft.set(20); 
		intakeRight.set(20);
	}
	public void reducedForward(){
		intakeLeft.set(10);
		intakeRight.set(10);
	}
	public void intakeReverse(){
		intakeLeft.set(-20);
		intakeRight.set(-20);
	}
	public void intakeStop(){
		intakeLeft.set(0);
		intakeRight.set(0);
	}
	public void debugValues(){
		SmartDashboard.putNumber(" Intake Left Current ", intakeLeft.getOutputCurrent());
		SmartDashboard.putNumber(" Intake Right Current ", intakeRight.getOutputCurrent());
		SmartDashboard.putNumber("Intake Left Error", intakeLeft.getSetpoint()-intakeLeft.getOutputCurrent());
		SmartDashboard.putNumber("Intake Right Error", intakeRight.getSetpoint()-intakeRight.getOutputCurrent());
		SmartDashboard.putNumber("Intake Left Voltage", intakeLeft.getOutputVoltage());
		SmartDashboard.putNumber("Intake Right Voltage", intakeRight.getOutputVoltage());
	}
	public void update(){
		pigeonUpdate();
		debugValues();
	}
	public void setPresetAngles(AnglePresets i){
		switch(i){
		case ZERO:
			_pidgey.SetFusedHeading(0);
			break;
		case ONE_EIGHTY:
			_pidgey.SetFusedHeading(180);
			break;
		case NINETY:
			_pidgey.SetFusedHeading(270);
			break;
		case TWO_SEVENTY:
			_pidgey.SetFusedHeading(90);
			break;
		}
	}
}
