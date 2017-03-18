package SubSystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
import com.ctre.PigeonImu;
import com.ctre.PigeonImu.PigeonState;

import Utilities.Ports;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {

	private static Intake instance = null;
	public CANTalon intakeMotor;
	private double currentAngle = 0.0;
    private PigeonImu _pidgey;
    boolean angleIsGood = false;
    double currentAngularRate = 0.0;
    public enum AnglePresets{
		ZERO,NINETY,ONE_EIGHTY, TWO_SEVENTY
	}
	public Intake(){
		intakeMotor = new CANTalon(Ports.INTAKE_MOTOR);
		intakeMotor.changeControlMode(TalonControlMode.Current);
		intakeMotor.setPID(0.01, 0.00, 0, 0.0175, 0, 0.0, 0);
		intakeMotor.reverseOutput(false);
		intakeMotor.setCloseLoopRampRate(24);
		try{
			_pidgey = new PigeonImu(intakeMotor);
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
			SmartDashboard.putNumber(" Pigeon Rate ", currentAngularRate);
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
		intakeMotor.set(20); 
	}
	public void reducedForward(){
		intakeMotor.set(10);
	}
	public void intakeReverse(){
		intakeMotor.set(-20);
	}
	public void intakeStop(){
		intakeMotor.set(0);
	}
	public void debugValues(){
		SmartDashboard.putNumber(" Intake Current ", intakeMotor.getOutputCurrent());
		SmartDashboard.putNumber("Intake Error", intakeMotor.getSetpoint()-intakeMotor.getOutputCurrent());
		SmartDashboard.putNumber("Intake Voltage", intakeMotor.getOutputVoltage());
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
	public void deployWings(){
		extendIntakes ex = new extendIntakes();
		ex.start();
	}
	public class extendIntakes extends Thread{
		double delay = 0.5;
		public void run(){
			intakeReverse();
			Timer.delay(delay);
			intakeStop();
		}
	}
}
