
package ControlSystem;

import SubSystems.GearIntake;
import SubSystems.Intake;
import SubSystems.Shooter;
import SubSystems.Sweeper;
import SubSystems.Swerve;
import SubSystems.Turret;
import Utilities.Ports;

public class RoboSystem{
    private static RoboSystem instance = null;
	public Intake intake;
    public Swerve dt;
//    public Navigation nav;
//	public Vision vision;
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
    	dt = Swerve.getInstance();
//    	nav = Navigation.getInstance();
//    	vision = Vision.getInstance();
    	shooter = Shooter.getInstance();
    	sweeper = Sweeper.getInstance();
    	turret = Turret.getInstance();
    	gearIntake = new GearIntake(Ports.GEAR_HANG, Ports.INTAKE_ARM);
    } 
}
