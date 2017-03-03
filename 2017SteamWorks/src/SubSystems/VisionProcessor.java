package SubSystems;


import ControlSystem.FSM;
import IO.Logger;

public class VisionProcessor implements Loop, VisionUpdateReceiver {
    static VisionProcessor instance_ = new VisionProcessor();
    VisionUpdate update_ = null;
    FSM fsm = FSM.getInstance();
    private static Logger logger = Logger.getInstance();
    public static VisionProcessor getInstance() {
        return instance_;
    }

    VisionProcessor() {
    }

    @Override
    public void onStart() {
    	logger.writeToLog("Processor starter");
    }

    @Override
    public void onLoop() {
        VisionUpdate update;
        synchronized (this) {
            if (update_ == null) {
                return;
            }
            update = update_;
            update_ = null;
        }
        
        fsm.addVisionUpdate(update.getCapturedAtTimestamp(), update.getTargets());
    }

    @Override
    public void onStop() {
        // no-op
    }

    @Override
    public synchronized void gotUpdate(VisionUpdate update) {
        update_ = update;
    }

}
