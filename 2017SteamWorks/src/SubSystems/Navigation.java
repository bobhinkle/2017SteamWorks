package SubSystems;

import Sensors.GyroThread;
import Utilities.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Navigation {
	
    protected GyroThread gyro;
    private double x = 0.0; // positive from driver facing center of the field
    private double y = 0.0; // positive from driver looking left
    private static Navigation instance;
    private double basicDistance = 0;
    private double angle = 0;
    private double STARTING_ANGLE_OFFSET = 0.0;
    private Navigation()
    {        
        gyro = new GyroThread();
        gyro.start();
    }
    public static Navigation getInstance()
    {
        if( instance == null )
        {
            instance = new Navigation();
        }
        return instance;
    }
    public void initGyro(){
        System.out.println("init");
        SmartDashboard.putString("GYRO_STATUS", "INITIALIZING");
        System.out.println("init done");
        SmartDashboard.putString("GYRO_STATUS", "READY");
    }
    public synchronized void resetRobotPosition(double x, double y, double theta,boolean gyroReset)
    {
        this.x = x;
        this.y = y;
        
        if(gyroReset){
            gyro.reset();
        }
        basicDistance = 0;
        STARTING_ANGLE_OFFSET = 0;
    }
    
    public synchronized double getX()
    {
        return x;
    }

    public synchronized double getY()
    {
        return y;
    }

    public double getHeadingInDegrees()
    {
        return Util.boundAngle0to360Degrees(gyro.getAngleInDegrees());
    }
    public double getRawHeading(){
        return gyro.getAngleInDegrees();
    }
    public double getRawHeadingInDegrees(){
    	return Util.radsToDegrees(angle);
    }

    public double getPitchInDegrees()
    {
        return 0;
    }

    public void resetPitch()
    {
        gyro.rezero();
    }

    public double getGyroRate(){
    	return gyro.getRate();
    }
    public double getDistance(){
        return basicDistance;
    }
    public void updatePosition()
    {
    	//angle = gyro.getAngleInDegrees() + STARTING_ANGLE_OFFSET;
    	//SmartDashboard.putNumber("GYRO_HEADING", gyro.getAngleInDegrees());
    }
    public double pidGet() {
        return getY();
    }
    
    
}