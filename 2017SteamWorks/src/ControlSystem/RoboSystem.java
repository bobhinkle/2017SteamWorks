
package ControlSystem;

import SubSystems.Navigation;
import SubSystems.Shooter;
import SubSystems.Swerve;
import SubSystems.Vision;

public class RoboSystem{
    private static RoboSystem instance = null;
    public Swerve dt;
    public Navigation nav;
	public Vision vision;
	public Shooter shooter;
    public static RoboSystem getInstance()
    {
        if( instance == null )
            instance = new RoboSystem();
        return instance;
    }
    
    public RoboSystem(){
    	dt = Swerve.getInstance();
    	nav = Navigation.getInstance();
    	vision = Vision.getInstance();
    	shooter = Shooter.getInstance();
    } 
}
