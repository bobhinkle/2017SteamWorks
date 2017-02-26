package ControlSystem;

import java.util.ArrayList;
import java.util.List;

import Helpers.InterpolatingDouble;
import Helpers.InterpolatingTreeMap;
import Helpers.RigidTransform2d;
import Helpers.Rotation2d;
import Helpers.Translation2d;
import IO.Logger;
import SubSystems.DistanceController;
import SubSystems.TargetInfo;
import Utilities.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FSM {

	public enum State{
    	DEFAULT, INIT,SHOOTER_STARTED,SHOOTER_WAITING,SHOOTER_READY,SHOOTER_SHOOTING, GEAR_GRAB, GEAR_SCORE, INTAKE_BALLS
    }
	private RoboSystem robot = RoboSystem.getInstance();
	private static FSM instance = null;
	public partsUpdate pu;
	private DistanceController dist;
	private State currentState = State.DEFAULT;
    protected InterpolatingTreeMap<InterpolatingDouble, RigidTransform2d> field_to_vehicle_;
    protected RigidTransform2d.Delta vehicle_velocity_;
    protected InterpolatingTreeMap<InterpolatingDouble, Rotation2d> turret_rotation_;
 //   protected GoalTracker goal_tracker_;
    protected Rotation2d camera_pitch_correction_;
    protected Rotation2d camera_yaw_correction_;
    protected double differential_height_;
    private Logger logger = Logger.getInstance();
    public static FSM getInstance()
    {
        if( instance == null )
            instance = new FSM();
        return instance;
    }
        
    public FSM() {
    	SmartDashboard.putString("FSM", "STARTED");
        pu = new partsUpdate();
    	pu.start();
    	dist = DistanceController.getInstance();
    }
   
    public class partsUpdate extends Thread{
        private boolean keepRunning = true;
    	public void run(){
    		SmartDashboard.putString("FSM", "THREAD STARTED");
    		while(keepRunning){
				update();
				Timer.delay(0.01); //10ms Loop Rate
    		} 
        }
        public void kill(){
        	keepRunning = false;
        }
    }

    public void update(){ 
//    	robot.vision.update();
        robot.dt.update();
        robot.intake.update();
        robot.sweeper.SweeperDebug();
        robot.turret.update();
        dist.update();
        robot.gearIntake.update();
        robot.shooter.update();
        
        switch(currentState){
        
        
        }
    }
    public void addVisionUpdate(double timestamp, List<TargetInfo> vision_update) {
    	
        List<Translation2d> field_to_goals = new ArrayList<>();
//        RigidTransform2d field_to_camera = getFieldToCamera(timestamp);
        if (!(vision_update == null || vision_update.isEmpty())) {
        	SmartDashboard.putBoolean("VisionUpdate", true);
            for (TargetInfo target : vision_update) {
                double ydeadband = (target.getY() > -Constants.kCameraDeadband
                        && target.getY() < Constants.kCameraDeadband) ? 0.0 : target.getY();

                // Compensate for camera yaw
                double xyaw = target.getX() * camera_yaw_correction_.cos() + ydeadband * camera_yaw_correction_.sin();
                double yyaw = ydeadband * camera_yaw_correction_.cos() - target.getX() * camera_yaw_correction_.sin();
                double zyaw = target.getZ();
                
                // Compensate for camera pitch
                double xr = zyaw * camera_pitch_correction_.sin() + xyaw * camera_pitch_correction_.cos();
                double yr = yyaw;
                double zr = zyaw * camera_pitch_correction_.cos() - xyaw * camera_pitch_correction_.sin();
                SmartDashboard.putNumber("TargetX", target.getX());
                SmartDashboard.putNumber("TargetY", target.getY());
                // find intersection with the goal
                if (zr > 0) {
                    double scaling = differential_height_ / zr;
                    double distance = Math.hypot(xr, yr) * scaling;
                    Rotation2d angle = new Rotation2d(xr, yr, true);
//                    field_to_goals.add(field_to_camera
 //                           .transformBy(RigidTransform2d
 //                                   .fromTranslation(new Translation2d(distance * angle.cos(), distance * angle.sin())))
 //                           .getTranslation());
                }
            }
        }else{
        	SmartDashboard.putNumber("TargetX", 0);
            SmartDashboard.putNumber("TargetY", 0);
            SmartDashboard.putBoolean("VisionUpdate", false);
            logger.writeToLog("Vision Update Fail");
        }
        synchronized (this) {
//            goal_tracker_.update(timestamp, field_to_goals);
        }
    }

}