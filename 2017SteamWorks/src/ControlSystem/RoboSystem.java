
package ControlSystem;

import SubSystems.*;
import Utilities.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; //added

public class RoboSystem{
    private static RoboSystem instance = null;
	public Intake intake;
    public Swerve dt;
	public Shooter shooter;
	public Sweeper sweeper;
	public Turret turret;
	public GearIntake gearIntake;
	public Hanger hanger;

    public static RoboSystem getInstance()
    {
        if( instance == null )
            instance = new RoboSystem();
        return instance;
    }
    
    public RoboSystem(){
    	
    	intake = Intake.getInstance();
//    	intake._pidgey.SetFusedHeading(-90);							// OVER HERE wasn't commented // that was a while ago
    	dt = Swerve.getInstance();
    	shooter = Shooter.getInstance();
    	sweeper = Sweeper.getInstance();
    	turret = Turret.getInstance();
    	gearIntake = new GearIntake(Ports.GEAR_INTAKE, Ports.INTAKE_ARM);
    	hanger = Hanger.getInstance();
    } 
}
