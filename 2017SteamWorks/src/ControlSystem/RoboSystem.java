
package ControlSystem;

import SubSystems.Intake;
import SubSystems.Swerve;

public class RoboSystem{
    private static RoboSystem instance = null;
    public Swerve dt;
//    public Navigation nav;
//	public Vision vision;
//	public Shooter shooter;
	public Intake intake;
    public static RoboSystem getInstance()
    {
        if( instance == null )
            instance = new RoboSystem();
        return instance;
    }
    
    public RoboSystem(){
    	dt = Swerve.getInstance();
//    	nav = Navigation.getInstance();
//    	vision = Vision.getInstance();
//    	shooter = Shooter.getInstance();
    	intake = Intake.getInstance();
    } 
}
