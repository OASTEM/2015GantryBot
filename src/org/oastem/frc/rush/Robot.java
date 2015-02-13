
package org.oastem.frc.rush;


import java.io.ObjectOutputStream.PutField;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;

import org.oastem.frc.Dashboard;
import org.oastem.frc.control.DriveSystem;
import org.oastem.frc.sensor.QuadratureEncoder;


public class Robot extends SampleRobot {
    
    // CHANGE THESE NUMBERS TO FINAL NUMBERS
    // MOTOR PORTS
    private static final int DRIVE_RIGHT_FRONT_PORT = 2;
    private static final int DRIVE_RIGHT_BACK_PORT = 3;
    private static final int DRIVE_LEFT_FRONT_PORT = 0;
    private static final int DRIVE_LEFT_BACK_PORT = 1;
    
    private static final int RIGHT_LIFT_PORT = 4;
    private static final int LEFT_LIFT_PORT = 3;
    
    private static final int RIGHT_ENC_I = 5;
    private static final int RIGHT_ENC_A = 4;
    private static final int RIGHT_ENC_B = 3;
    private static final int LEFT_ENC_I = 1;
    private static final int LEFT_ENC_A = 2;
    private static final int LEFT_ENC_B = 0;
    
    // MOTORS
    private Victor rightDriveFront;
    private Victor rightDriveBack;
    private Victor leftDriveFront;
    private Victor leftDriveBack;
    
    private CANJaguar rightLift;
    private CANJaguar leftLift;
    
    // JOYSTICK BUTTONS
    private static final int LIFT_UP = 6;
    private static final int LIFT_DOWN = 7;
    private static final int LIFT_MODE_TOGGLE = 2;
    
    //CONSTANTS
    private static final int PLAN_ENC_CPR = 497;
    private static final int LIFT_HEIGHT_LIMIT = 40;
    private static final double LIFT_GRAD_DISTANCE = .001;
    private static final double LIFT_MOVE_DISTANCE = 15; // WE MIGHT NEED TO FINE TUNE THIS
    
    // 				WE NEED TO CHECK THIS DISTANCE
    private static final double DISTANCE_PER_REV = 2.5 * Math.PI;
    
    //DECLARING OBJECTS
    private DriveSystem drive = DriveSystem.getInstance();
    private Joystick joystick;
    private Dashboard dash;
    private QuadratureEncoder rightEnc;
    private QuadratureEncoder leftEnc;
    
    //AUTONOMOUS STATES
    public static final int START = 0;
	public static final int GOTO_TOTE = 1;
	public static final int UPLIFT = 2;
	public static final int RESET = 3;
	public static final int MOVETO_AUTO = 4;
	public static final int RELEASE = 5;
	//public static final int READY = 6;

    public void robotInit() {
        //drive.initializeDrive(DRIVE_LEFT_FRONT_PORT, DRIVE_LEFT_BACK_PORT, DRIVE_RIGHT_FRONT_PORT, DRIVE_RIGHT_BACK_PORT);
        drive.initializeDrive(DRIVE_LEFT_BACK_PORT, DRIVE_RIGHT_BACK_PORT);
        drive.setSafety(false);
    	/*rightDriveFront = new Victor(DRIVE_RIGHT_FRONT_PORT);
        //rightDriveBack = new Victor(DRIVE_RIGHT_BACK_PORT);
        //leftDriveFront = new Victor(DRIVE_LEFT_FRONT_PORT);
        leftDriveBack = new Victor(DRIVE_LEFT_BACK_PORT);
        */
        //drive.setInvertedDouble();
        
        rightEnc = new QuadratureEncoder(RIGHT_ENC_A, RIGHT_ENC_B, RIGHT_ENC_I);
        leftEnc = new QuadratureEncoder(LEFT_ENC_A, LEFT_ENC_B, LEFT_ENC_I);
        /*
        rightLift = new CANJaguar(RIGHT_LIFT_PORT);
        leftLift = new CANJaguar(LEFT_LIFT_PORT);
        
        rightLift.setPositionMode(CANJaguar.kQuadEncoder, PLAN_ENC_CPR, 1000, 0.002, 1000);
        rightLift.configForwardLimit(-LIFT_HEIGHT_LIMIT/DISTANCE_PER_REV);
        rightLift.configReverseLimit(1);
        rightLift.enableControl(0);
        
        leftLift.setPositionMode(CANJaguar.kQuadEncoder, PLAN_ENC_CPR, 1000, 0.002, 1000);
        leftLift.configForwardLimit(LIFT_HEIGHT_LIMIT/DISTANCE_PER_REV);
        leftLift.configReverseLimit(-1);
        leftLift.enableControl(0);
        */
        joystick = new Joystick(0);
        
        dash = new Dashboard();
    }

    /**
     * Auto from before
     *
    public void autonomous() {
    	
    	double liftPosition = 0;
    	int state = 0;
    	int resetCount = 0;
    	long currTime = 0L;
    	long triggerStart = 0L;
    	
    	while(isAutonomous() && isEnabled()) {
			currTime = System.currentTimeMillis();
			//joytonomousStates(currTime); //not sure if this should be here
		}
    	
    }

    /*
	*This is autonomous pseudocode and it should hopefully logically work <3
	
	public static final int START = 0;
	public static final int GOTO_TOTE = 1;
	public static final int UPLIFT = 2;
	public static final int RESET = 3;
	public static final int MOVETO_AUTO = 4;
	public static final int RELEASE = 5;
	//public static final int READY = 6;
	
	private int state = 0;
	private int resetCount = 0;
	private long currTime = 0L;
	private long triggerStart = 0L;
	
	public void autonomous() {
		debug[0] = "Autonomous mode enabled";
		while(isAutonomous() && isEnabled()) {
			//imageProcessing();
			currTime = System.currentTimeMillis();
			joytonomousStates(currTime); 
		}
		Debug.log(debug);
	}
	
	private void joytonomousStates(long currTime) {
		switch(state) {
			case START:
				//anything we need to go beforehand
				state = GOTO_TOTE;
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
					} 
					if (currTime - triggerStart > 1750L) { // more adjust time as necessary
						triggerStart = currTime;
						state = RESET; //count attempts
						resetCount++;
					}
				} else {
					triggerStart = currTime
					state = MOVETO_AUTO;
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
				} 
				if (currTime - triggerStart > 5000L) { // adjust time as necesary
					triggerStart = currTime;
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
				System.out.println("Kms");
				break; //ihy
		}
	}
	
	private boolean moveForward(long currTime, long triggerStart) {
		if(!robot.drive(distanceToTote) && currTime - triggerStart <= 1500L) { //lol this isn't a method but should return T/F 
			return false;
		} else {
			return true;
		}
	}
	
	private boolean hookUp(long currTime, long triggerStart) {
		hook.upToTote(); //lol I wish this were already a method
		if(!checkHooked()) { //however long it takes to hook the tote
			return false;
		} else {
			return true;
		}
	}
	
	private boolean redo(long currTime, long triggerStart) {
		hook.downToUnhook();
		if (!robot.drive(justSmallDistanceToReposition) && currTime - triggerStart <= 1000L) {
			return false;
		} else {
			return true;
		}
	}
	
	private boolean moveAuto(long currTime, long triggerStart) {
		if(!robot.drive(distanceToAuto)) {
			return false;
		} else {
			return true;
		}
	}
	
	private boolean releaseTote(long currTime, long triggerStart) {
		hook.downToUnhook();
		if(checkHooked() && currTime - triggerStart <= 1500L) {
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
    	
    	double liftPosition = 0;
    	boolean canPressToggle = true;
    	boolean isIncrement = true;
    	
    	rightEnc.reset();
    	leftEnc.reset();
    	dash.putString("Lift Mode: ", "INCREMENT");
    	
        while (isOperatorControl() && isEnabled()) {
            drive.arcadeDrive(joystick);
            dash.putNumber("Right Drive: ", rightEnc.getRaw());
            dash.putNumber("Left Drive : ", leftEnc.getRaw());
            /*
            //MOVE LIFT IN INCREMENTS
            if (joystick.getRawButton(LIFT_UP) && isIncrement)
            {
            	liftPosition = LIFT_MOVE_DISTANCE;
            }
            else if (joystick.getRawButton(LIFT_DOWN) && isIncrement)
            {
            	liftPosition = 0;
            }
            
            
            //MOVE LIFT GRADUALLY
            if (joystick.getRawButton(LIFT_UP)  && !isIncrement)
            {
            	liftPosition += LIFT_GRAD_DISTANCE;
            }
            else if (joystick.getRawButton(LIFT_DOWN) && !isIncrement)
            {
            	liftPosition -= LIFT_GRAD_DISTANCE;
            }
            
            //TOGGLE BETWEEN LIFT MODES
            if (joystick.getRawButton(LIFT_MODE_TOGGLE) && canPressToggle)
            {
            	if (isIncrement)
            	{
            		isIncrement = false;
            		dash.putString("Lift Mode: ", "GRADUAL");
            	}
            	else
            	{
            		isIncrement = true;
            		dash.putString("Lift Mode: ", "INCREMENT");
            	}
            	
            	canPressToggle = false;
            }
            
            //MAKE SURE BUTTON NOT PRESSED
            if (!joystick.getRawButton(LIFT_MODE_TOGGLE))
            	canPressToggle= true;
            
            
            this.setLift(liftPosition);
            
            
            if ((liftPosition - leftLift.getPosition()) > 0)
            	dash.putString("Lift: ", "UP");
            else if ((liftPosition - leftLift.getPosition()) < 0)
            	dash.putString("Lift: ", "DOWN");
            else
            	dash.putString("Lift: ", "STOPPED");
            
            */
            Timer.delay(0.005);		// wait for a motor update time
        }
    }

    private void setLift(double setPoint){
    	rightLift.set(-setPoint / DISTANCE_PER_REV); // right is reflected
    	leftLift.set(setPoint / DISTANCE_PER_REV);
    }
    
    private void moveDistance(double distance){
    	
    }
    
    /**
     * Runs during test mode
     */
    public void test() {
    }
}
