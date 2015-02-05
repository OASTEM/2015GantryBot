
package org.oastem.frc.rush;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;

import org.oastem.frc.Dashboard;
import org.oastem.frc.control.DriveSystem;


public class Robot extends SampleRobot {
    private DriveSystem drive = DriveSystem.getInstance();
    private Joystick joystick;
    
    // CHANGE THESE NUMBERS TO FINAL NUMBERS
    private static final int DRIVE_RIGHT_FRONT = 0;
    private static final int DRIVE_RIGHT_BACK = 1;
    private static final int DRIVE_LEFT_FRONT = 2;
    private static final int DRIVE_LEFT_BACK = 3;
    
    private Victor rightDriveFront;
    private Victor rightDriveBack;
    private Victor leftDriveFront;
    private Victor leftDriveBack;
    
    public static final int START = 0;
	public static final int GOTO_TOTE = 1;
	public static final int UPLIFT = 2;
	public static final int RESET = 3;
	public static final int MOVETO_AUTO = 4;
	public static final int RELEASE = 5;
	//public static final int READY = 6;

    public void robotInit() {
        drive.initializeDrive(DRIVE_LEFT_FRONT, DRIVE_LEFT_BACK, DRIVE_RIGHT_FRONT, DRIVE_RIGHT_BACK);
        rightDriveFront = new Victor(DRIVE_RIGHT_FRONT);
        rightDriveBack = new Victor(DRIVE_RIGHT_BACK);
        leftDriveFront = new Victor(DRIVE_LEFT_FRONT);
        leftDriveBack = new Victor(DRIVE_LEFT_BACK);
        
        joystick = new Joystick(0);
    }

    /**
     * AUTONOMOUS
     */
    public void autonomous() {
    	
    	
    	int state;
    	int resetCount;
    	long currTime = 0L;
    	long triggerStart = 0L;
    	
    	while(isAutonomous() && isEnabled()) {
			currTime = System.currentTimeMillis();
			//joytonomousStates(currTime); //not sure if this should be here
		}
    	
    }

    /*
    private void joytonomousStates(long currTime) {
		switch(state) {
			case START:
				//anything we need to go beforehand
				break;
			case GOTO_TOTE:
				if(moveForward(currTime, triggerStart)) {
					triggerStart = currTime;
					state = UPLIFT;
				}
				break;
			case UPLIFT:
				if(resetCount < 3) {
					if(hookUp(currTime, triggerStart)) {
						triggerStart = currTime;
						state = MOVETO_AUTO;
					} else {
						state = RESET; //count attempts
						resetCount++;
					}
				}
				break;
			case RESET:
				if(redo(currTime, triggerStart)) {
					triggerStart = currTime;
					state = UPLIFT;
				}
				break;
			case MOVETO_AUTO;
				if(moveAuto(currTime, triggerStart)) {
					triggerStart = currTime;
					state = RELEASE;
				} else {
					state = READY;
				}
				break;
			case RELEASE:
				if(releaseTote(currTime, triggerStart)) {
					triggerStart = 0;
					state = READY;
				}
			case READY:
				//how we want to get ready for operator control
			default: 
				//what to do if something fails
				break; //ihy
		}
	}
	
	private boolean moveForward(long currTime, long triggerStart) {
		if(!robot.drive(distanceToTote) || currTime - triggerStart > 1500L) { //lol this isn't a method but should return T/F 
			return false;
		} else {
			return true;
		}
	}
	
	private boolean hookUp(long currTime, long triggerStart) {
		hook.upToTote(); //lol I wish this were already a method
		if(!checkHooked() || currTime - triggerStart > 2000L ) { //however long it takes to hook the tote
			return false;
		} else {
			return true;
}
	}
	
	private boolean redo(long currTime, long triggerStart) {
		hook.downToUnhook();
		if (!robot.drive(justSmallDistanceToReposition) || currTime - triggerStart > 1000L) {
			return false;
		} else {
			return true;
		}
	}
	
	private boolean moveAuto(long currTime, long triggerStart) {
		if(!robot.drive(distanceToAuto) || !checkHooked() || currTime-triggerStart > 5000L) {
			return false;
		} else {
			return true;
		}
	}
	
	private boolean releaseTote(long currTime, long triggerStart) {
		hook.downToUnhook();
		if(checkHooked() || currTime - triggerStart > 1500L) {
			return false;
		} else {
			robot.reverse(justSmallDistanceToReposition);
			return true;
		}
	}
	
	*/
    
    
    /**
     * Runs the motors with arcade steering.
     */
    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {
            drive.arcadeDrive(joystick);
            Timer.delay(0.005);		// wait for a motor update time
        }
    }

    /**
     * Runs during test mode
     */
    public void test() {
    }
}
