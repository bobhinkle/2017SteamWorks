package SubSystems;

import Utilities.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Navigation {
	
    private double x = 0.0; // positive from driver facing center of the field
    private double y = 0.0; // positive from driver looking left
    private static Navigation instance;
    private double basicDistance = 0;
    private double angle = 0;
    private double STARTING_ANGLE_OFFSET = 0.0; //
    private Swerve swerve;
    private Intake intake;
    private double currentAngle = 0.0;
    double currentAngularRate = 0.0;
    private Navigation()
    {        
        intake = Intake.getInstance();		
    }
    public static Navigation getInstance()
    {
        if( instance == null )
        {
            instance = new Navigation();
        }
        return instance;
    }
    
    public double getCurrentAngle(){
    	return currentAngle;
    }
    public double getCurrentAngularRate(){
    	return currentAngularRate;
    }
    public synchronized void resetRobotPosition(double x, double y, double theta,boolean gyroReset)
    {
        this.x = x;
        this.y = y;
        
       
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
    	return 0;
//        return Util.boundAngle0to360Degrees(gyro.getAngleInDegrees());
    }
    public double getRawHeading(){
        return 0;
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
        
    }

    public void moduleCoords(){
    	SmartDashboard.putNumber("LMOD_X", swerve.frontLeft.getX());
    	SmartDashboard.putNumber("LMOD_Y", swerve.frontLeft.getY());
    }
    
    public double getDistance(){
        return basicDistance;
    }
    public void updatePosition()
    {
//    	pigeonUpdate();
    	//angle = gyro.getAngleInDegrees() + STARTING_ANGLE_OFFSET;
    	//SmartDashboard.putNumber("GYRO_HEADING", gyro.getAngleInDegrees());
    }
    public double pidGet() {
        return getY();
    }
    
    
}