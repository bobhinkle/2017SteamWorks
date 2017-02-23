package Utilities;

/**
 *
 * @author Rohi Zacharia
 */
public class Constants {
    
    public static final boolean LOW_GEAR  = true; //Drivetrain low gear
    public static final double MIN_DT_POWER = 0.2;
    public static final double STARTING_ANGLE_OFFSET = 0;
    
    public static final int GYRO_INIT = 0;
    public static final int GYRO_READY  = 1;
    
    public static final double WHEELBASE_LENGTH = 22.181;
    public static final double WHEELBASE_WIDTH  = 15.681;
    public static final double SWERVE_R = 27.16;
	public static final double ANGLE_FRONT_MODULE_CENTER = Math.atan(WHEELBASE_LENGTH/WHEELBASE_WIDTH);
	
    
    public static final double FRONT_RIGHT_TURN_OFFSET = 283.0; //281.2
    public static final double FRONT_LEFT_TURN_OFFSET  = 170.5; //171.2
    public static final double REAR_LEFT_TURN_OFFSET   = 138.1; //289.3
    public static final double REAR_RIGHT_TURN_OFFSET  = 170.1; //172.6
    
    public static final double DRIVE_TICKS_PER_INCH =  (13*5760)/(16*Math.PI)/2.0;//36/25542; //0.00200635031508792675265469178699;//0.00163990667972621570986118595697; //0.00150603674668734095803578302171;//60.0/40462.0; //
    public static final double TURN_KP = 0.02; //0.020
    public static final double TURN_KI = 0.00;
    public static final double TURN_KD = 0.02;//0.02
    public static final double TURN_KFV = 0.0000; //0.0001
    public static final double TURN_KFA = 0.0000; //0.0001
    public static final double TURN_ON_TARGET_DEG = 1;
    public static final double TURN_MAX_ACCEL = 15.0; //25
    public static final double TURN_MAX_VEL = 600.0; //900
    public static final double MAX_ROTATION_ANGLE_PER_SEC = 2;
    // Turn, KP, turn!
    public static final double TURN_KP_TURN = 0.08;
    
	//Swerve Turning Gains
	public static final double SWERVE_TURNING_GAIN_P = 0.02; // percent throttle per degree of error 0.02
	public static final double SWERVE_TURNING_GAIN_D = 0.00425; // percent throttle per angular velocity dps 0.00425
	public static final double SWERVE_SMALL_TURNING_GAIN_P = 0.009; //0.015
	public static final double SWERVE_SMALL_TURNING_GAIN_D = 0.002; //0.002
	// TODO Tune kPHeadingGain (now SWERVE_HEADING_GAIN_P) to prevent drift while driving
	public static final double SWERVE_HEADING_GAIN_P = 0.003; //0.002
	public static final double SWERVE_HEADING_GAIN_D = 0.001;
	public static final double SWERVE_HEADING_MAX_CORRECTION_RATIO = 0.75; //0.75
	public static final double SWERVE_SMALL_HEADING_MAX_CORRECTION_RATIO = 0.18; //0.18
	public static final double SWERVE_MAX_CORRECTION_HEADING = 0.12;
	public static final double SWERVE_ROTATION_SCALE_FACTOR = 0.225;
	public static final double SWERVE_ROTATION_SCALE_FACTOR_FAST = 0.35;
    
    public static final double TURNING_ADD_POWER_THRESHOLD = 10; //10
    public static final double TURNING_DETECT_THRESHOLD = TURNING_ADD_POWER_THRESHOLD;//1; //10// 3
    public static final int DRIVING_DETECT_THRESHOLD = 1; // clicks
    public static final double ROBOT_ROTATING_DETECT_THRESHOLD = 15;
	public static final int MIN_CYCLES_HEADING_ON_TARGET = 30;
	public static final double HEADING_MAX_ERROR = 1.0;
	public static final double SWERVE_ROTATION_HEADING_ON_TARGET_THRESHOLD = /**/HEADING_MAX_ERROR/*/10/**/;
    
    public static final double CAMERA_PIXEL_WIDTH = 320.0;
	public static final double CAMERA_FOV = 60; //45.3
	public static final double GRIP_X_OFFSET = -0.5; // negative to go left, positive to go right. 1.75
	public static final double CAM_CALIBRATION = 1.0;
	
	public static final double TURRET_CLICKS_TO_ANGLE = 66.2252;
	public static final double TURRET_MAX_ANGLE = 90;
	
	public static final double STICK_DEAD_BAND = 0.2;
	
	public static final double SHOOTING_SPEED = 4000;
	public static final double SHOOTER_ERROR  = 100;
	
	public static final double GEAR_INTAKE_POWER = 1.0;
	public static final double GEAR_INTAKE_CURR_DETECT = 50;
	
	//Distance Controller
	public static final double DIST_CONTROLLER_P = 0.000040; //0.0000[23]5
	public static final double DIST_CONTROLLER_D = 0.00001;
	public static final double DIST_CONTROLLER_SMALL_P = 0.00003; //0.00003
	public static final double DIST_CONTROLLER_SMALL_D = 0.00002; //0
	public static final double DIST_CONTROLLER_PID_THRESH = 5.0;
	public static final int DIST_CONTROLLER_CYCLE_THRESH = 15;

	public static final double RADIUS_CENTER_TO_MODULE = Math.sqrt(Math.pow(WHEELBASE_LENGTH/2, 2)+Math.pow(WHEELBASE_WIDTH/2, 2))*DRIVE_TICKS_PER_INCH;
	
	
}