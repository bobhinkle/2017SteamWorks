/**
 * 
 */
package SubSystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The LightController subsystem controls the signal sent to the Arduino board, which lights up LEDs to signal various robot states.
 * 
 * @author Joseph Reed
 */
public class LightController {
	
	private static LightController instance = new LightController();	// Eclipse wanted to add the `static'
	public static LightController getInstance() {
		if(instance == null)
				instance = new LightController();
		return instance;
	}
	public enum Status {
		/**  */
		OFF,
		/** Robot is disabled. */
		DISABLED,
		/** Pink means catastrophic failure! */
		CATASTROPHE,
		/** {@link SubSystems.GearIntake GearIntake} reports that a gear was taken in. */
		GEAR_DETECTED,
		/** {@link SubSystems.GearIntake GearIntake} reports that a gear was lost. */
		GEAR_LOST,
		/** {@link ControlSystem.FSM FSM} reports that the boiler is visible. */
		TARGET_DETECTED,
		/** {@link ControlSystem.FSM FSM} reports that the boiler is not visible. */
		TARGET_NOT_DETECTED
	}
	/** The current status being reported by the {@link LightController}. */
	private Status status;
	
	public LightController() {
		status = Status.OFF;
	}
	public void setStatus(Status _status) {
		status = _status;
	}
	public void update() {
		switch(status) {
			case OFF:
				SmartDashboard.putString("Lights", "0");
				break;
			case DISABLED: 
				SmartDashboard.putString("Lights", "2");
				break;
			case CATASTROPHE: 
				SmartDashboard.putString("Lights", "7");
				break;
			case GEAR_DETECTED:
				SmartDashboard.putString("Lights", "6");
				break;
			case GEAR_LOST:
				SmartDashboard.putString("Lights", "3");
				break;
			case TARGET_DETECTED: 
				SmartDashboard.putString("Lights", "4");
				break;
			case TARGET_NOT_DETECTED:
				SmartDashboard.putString("Lights", "1");
				break;
			default:
				SmartDashboard.putString("Lights", "5"); // if we're not disabled but nothing else is happening?
				break;
		}
	}
}
