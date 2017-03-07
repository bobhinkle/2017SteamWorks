/**
 * 
 */
package SubSystems;

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
		/** "Black" means the robot is off. */
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
			case OFF: break;
			case DISABLED: break;
			case CATASTROPHE: break;
			case GEAR_DETECTED: break;
			case GEAR_LOST: break;
			case TARGET_DETECTED: break;
			case TARGET_NOT_DETECTED: break;
			default: break;
		}
	}
}
