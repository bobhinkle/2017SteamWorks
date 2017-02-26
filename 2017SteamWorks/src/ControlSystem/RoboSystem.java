
package ControlSystem;

import SubSystems.*;
import Utilities.Ports;

public class RoboSystem{
    private static RoboSystem instance = null;
	public Intake intake;
    public Swerve dt;
	public Shooter shooter;
	public Sweeper sweeper;
	public Turret turret;
	public GearIntake gearIntake;

    public static RoboSystem getInstance()
    {
        if( instance == null )
            instance = new RoboSystem();
        return instance;
    }
    
    public RoboSystem(){
    	
    	intake = Intake.getInstance();
//    	intake._pidgey.SetFusedHeading(-90);							// OVER HERE wasn't commented
    	dt = Swerve.getInstance();
    	shooter = Shooter.getInstance();
    	sweeper = Sweeper.getInstance();
    	turret = Turret.getInstance();
    	gearIntake = new GearIntake(Ports.GEAR_HANG, Ports.INTAKE_ARM);
    } 
}
