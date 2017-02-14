package SubSystems;        

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import Utilities.Ports;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    private static Shooter instance = null;
    public static enum Status{
		OFF, FILLING_NO_BALL, FILLED, SHOOTING
	}
    public class ShooterColumn{
    	private CANTalon motor;
    	private int absolutePosition;
    	Shooter.Status status = Shooter.Status.OFF;
    	public ShooterColumn(int motor_id,boolean reversed){
    		motor = new CANTalon(motor_id);
        	absolutePosition = motor.getPulseWidthPosition() & 0xFFF;
        	motor.setEncPosition(absolutePosition);
        	motor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        	motor.reverseSensor(reversed);
        	motor.reverseOutput(true);
        	motor.configEncoderCodesPerRev(360);
        	motor.configNominalOutputVoltage(+0f, -0f);
        	motor.configPeakOutputVoltage(0, -12f);
        	motor.setAllowableClosedLoopErr(0); 
        	motor.changeControlMode(TalonControlMode.Speed);
        	motor.set(0);        	
        	//motor1.setPID(0.1, 0.0, 0.4, 0.050, 0, 0.0, 0);
        	//motor1.setPID(0.0, 0.0, 0.0, 0.05, 0, 0.0, 1);        	
    	}
    	public void setSpeed(double speed){
    		motor.set(speed);
    	}
    	public void update(){
    		switch(status){
    		case SHOOTING:
    			motor.set(1000);
    			break;  
    			default:
//    				motor1.set(0);
//   				motor2.set(0);
//    				preloader.set(0);
    				break;
    		}
    		SmartDashboard.putNumber("LeftShooter", leftShooter.motor.getSpeed());
    		SmartDashboard.putNumber("LEFT_SET", leftShooter.motor.getSetpoint());
    		SmartDashboard.putNumber("RightShooter", rightShooter.motor.getSpeed());
    		SmartDashboard.putNumber("Right_SET", rightShooter.motor.getSetpoint());
    	}
    }
    public void setSpeed(double speed){
    	leftShooter.setSpeed(speed);
    	rightShooter.setSpeed(speed);
    }
    public void bumpUp(double increase){
    	leftShooter.setSpeed(leftShooter.motor.get() + increase);
    	rightShooter.setSpeed(leftShooter.motor.get() + increase);
    }
    public void bumpDown(double decrease){
    	leftShooter.setSpeed(leftShooter.motor.get() - decrease);
    	rightShooter.setSpeed(leftShooter.motor.get() - decrease);
    }
    public ShooterColumn leftShooter,rightShooter;
	public Shooter(){
		leftShooter = new ShooterColumn(Ports.SHOOTER_MOTOR_MASTER,true);
		rightShooter = new ShooterColumn(Ports.SHOOTER_MOTOR_SLAVE,false);
	}
	public static Shooter getInstance()
    {
        if( instance == null )
            instance = new Shooter();
        return instance;
    }
}
