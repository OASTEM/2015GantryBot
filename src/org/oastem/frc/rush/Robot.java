
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


public class Robot extends SampleRobot {
    
    // CHANGE THESE NUMBERS TO FINAL NUMBERS
    // MOTOR PORTS
    private static final int DRIVE_RIGHT_FRONT_PORT = 0;
    private static final int DRIVE_RIGHT_BACK_PORT = 1;
    private static final int DRIVE_LEFT_FRONT_PORT = 2;
    private static final int DRIVE_LEFT_BACK_PORT = 3;
    
    private static final int RIGHT_LIFT_PORT = 4;
    private static final int LEFT_LIFT_PORT = 5;
    
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
    private static final int BIN_BUTTON = 4;
    private static final int TOTE_BUTTON = 5;
    private static final int RESET_BUTTON = 3;
    private static final int MAN_BUTTON = 10;
    private static final int EXIT_MAN_BUTTON = 11;
    
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
    
    //AUTONOMOUS STATES
    public static final int START = 0;
	public static final int GOTO_TOTE = 1;
	public static final int UPLIFT = 2;
	public static final int REDO = 3;
	public static final int MOVETO_AUTO = 4;
	public static final int RELEASE = 5;
	//public static final int READY = 6;
	
	// CLAW HEIGHTS
	private static final int BOTTOM = 0;
	private static final int ABOVE_TOTE = 1;
	private static final int TOTE_DRIVE = 2;
	private static final int SECOND_TOTE = 3;
	private static final int GRAB_BIN = 4;
	private static final int BIN_DRIVE = 5;
	private static final int BIN_TOTE = 6;
	
	// USERCONTROL STATES
	private static final int RESET = 0;
	private static final int READY_FOR_TOTE = 1;
	private static final int GRABBING_TOTE = 2;
	private static final int TOTE_GRABBED = 3;
	private static final int READY_FOR_NEXT = 4;
	private static final int READY_FOR_BIN = 5;
	private static final int BIN_GRABBED = 6;
	private static final int READY_FOR_BIN_TOTE = 7;
	private static final int MANUAL = 8;
	

    public void robotInit() {
        drive.initializeDrive(DRIVE_LEFT_FRONT_PORT, DRIVE_LEFT_BACK_PORT, DRIVE_RIGHT_FRONT_PORT, DRIVE_RIGHT_BACK_PORT);
        /*rightDriveFront = new Victor(DRIVE_RIGHT_FRONT_PORT);
        rightDriveBack = new Victor(DRIVE_RIGHT_BACK_PORT);
        leftDriveFront = new Victor(DRIVE_LEFT_FRONT_PORT);
        leftDriveBack = new Victor(DRIVE_LEFT_BACK_PORT);
        */
        rightLift = new CANJaguar(RIGHT_LIFT_PORT);
        leftLift = new CANJaguar(LEFT_LIFT_PORT);
        
        initRightLift();
        initLeftLift();
        
        joystick = new Joystick(0);
        
        dash = new Dashboard();
    }
    
    private void initRightLift()
    {
    	rightLift.setPositionMode(CANJaguar.kQuadEncoder, PLAN_ENC_CPR, 1000, 0.002, 1000);
        rightLift.configForwardLimit(-LIFT_HEIGHT_LIMIT/DISTANCE_PER_REV);
        rightLift.configReverseLimit(1);
        rightLift.enableControl(0);
        
    }
    private void initLeftLift()
    {
        leftLift.setPositionMode(CANJaguar.kQuadEncoder, PLAN_ENC_CPR, 1000, 0.002, 1000);
        leftLift.configForwardLimit(LIFT_HEIGHT_LIMIT/DISTANCE_PER_REV);
        leftLift.configReverseLimit(-1);
        leftLift.enableControl(0);
    }

    /**
     * AUTONOMOUS
     */
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
			case REDO:
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
		hook.upToTote(); //lol I wish this were already a method // Instead of method, increase position
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
    	
    	double liftPosition = 0;
    	boolean canPressToggle = true;
    	boolean isIncrement = true;
    	int state = 0;
    	int saveState = 0;
    	
    	dash.putString("Lift Mode: ", "INCREMENT");
    	
        while (isOperatorControl() && isEnabled()) {
            //drive.arcadeDrive(joystick);
            
        	// CHECK THE JAGUAR FOR RIGHT CAN ID
        	
        	// CHECK THE LIMIT SWITCH
        	/*
        	rightLift.setPercentMode();
        	rightLift.enableControl();
        	rightLift.set(joystick.getX());
        	dash.putBoolean("Limit Switch touched?", rightLift.getReverseLimitOK());
        	/////////////*/
            
        	// SPRING
        	// YO SPRING ACTUALLY READ THIS
        	// MAKE SURE TO BE PREPARED TO DISABLE
        	/*
        	rightLift.configForwardLimit(5);
        	rightLift.set(10);
        	//*/
        	
        	// FIGURE OUT WHICH LIFT SIDE HAS TO BE REFLECTED
        	// CHANGE INIT AND SET
        	
            /*
            dash.putString("Status: ", "Calibrating...");
            switch(state)
            {
            case RESET :
            	if (calibratedLift())
            	{
            		dash.putString("Status: ", "in RESET");
            		if (joystick.getRawButton(TOTE_BUTTON))
            			state = READY_FOR_TOTE;
            		if (joystick.getRawButton(BIN_BUTTON))
            			state = READY_FOR_BIN;
            	}
            	break;
            case READY_FOR_TOTE :
            	dash.putString("Status: ", "in READY_FOR_TOTE");
            	setLift(ABOVE_TOTE);
            	if (joystick.getRawButton(LIFT_UP) && rightLift.getPosition() == -ABOVE_TOTE && leftLift.getPosition() == ABOVE_TOTE)
            		state = GRABBING_TOTE;
            	if (joystick.getRawButton(BIN_BUTTON))
        			state = READY_FOR_BIN;
            	break;
            case READY_FOR_BIN :
            	dash.putString("Status: ", "in READY_FOR_BIN");
            	setLift(GRAB_BIN);
            	if (joystick.getRawButton(LIFT_UP) && rightLift.getPosition() == -GRAB_BIN && leftLift.getPosition() == GRAB_BIN)
            		state = BIN_GRABBED;
            	if (joystick.getRawButton(TOTE_BUTTON))
        			state = READY_FOR_TOTE;
            	break;
            case GRABBING_TOTE :
            	dash.putString("Status: ", "in GRABBING_TOTE");
            	setLift(BOTTOM);
            	if (joystick.getRawButton(LIFT_UP) && rightLift.getPosition() == -BOTTOM && leftLift.getPosition() == BOTTOM)
            		state = TOTE_GRABBED;
            	if (joystick.getRawButton(BIN_BUTTON))
        			state = READY_FOR_BIN;
            	if (joystick.getRawButton(TOTE_BUTTON))
        			state = READY_FOR_TOTE;
            	break;
            case TOTE_GRABBED :
            	dash.putString("Status: ", "in TOTE_GRABBED");
            	setLift(TOTE_DRIVE);
            	if (rightLift.getPosition() == -TOTE_DRIVE && leftLift.getPosition() == TOTE_DRIVE)
            	{
            		if (joystick.getRawButton(LIFT_UP))
            			state = READY_FOR_NEXT;
            		else if (joystick.getRawButton(LIFT_DOWN))
            			state = GRABBING_TOTE;
            	}
            	break;
            case BIN_GRABBED :
            	dash.putString("Status: ", "in BIN_GRABBED");
            	setLift(BIN_DRIVE);
            	if (rightLift.getPosition() == -BIN_DRIVE && leftLift.getPosition() == BIN_DRIVE)
            	{
            		if ((joystick.getRawButton(LIFT_UP) || joystick.getRawButton(TOTE_BUTTON)))
            			state = READY_FOR_BIN_TOTE;
            		else if (joystick.getRawButton(LIFT_DOWN))
            			state = READY_FOR_BIN;
            	}
            	break;
            case READY_FOR_NEXT :
            	dash.putString("Status: ", "in READY_FOR_NEXT");
            	setLift(SECOND_TOTE);
            	if(rightLift.getPosition() == -SECOND_TOTE && leftLift.getPosition() == SECOND_TOTE)
            	{
            		if (joystick.getRawButton(LIFT_UP))
            			state = GRABBING_TOTE;
            		else if (joystick.getRawButton(LIFT_DOWN))
            			state = TOTE_GRABBED;
            	}
            	break;
            case READY_FOR_BIN_TOTE :
            	dash.putString("Status: ", "in READY_FOR_BIN_TOTE");
            	setLift(BIN_TOTE);
            	if(rightLift.getPosition() == -BIN_TOTE && leftLift.getPosition() == BIN_TOTE)
            	{
            		if (joystick.getRawButton(LIFT_UP))
            			state = GRABBING_TOTE;
            		else if (joystick.getRawButton(LIFT_DOWN))
            			state = BIN_GRABBED;
            	}
            	break;
            case MANUAL :
            	dash.putString("Status: ", "EMANUEL");
            	if (joystick.getRawButton(LIFT_UP))
            		setLift(Math.abs(rightLift.get()) + LIFT_GRAD_DISTANCE);
            	else if (joystick.getRawButton(LIFT_DOWN))
            		setLift(Math.abs(rightLift.get()) - LIFT_GRAD_DISTANCE);
            		
            	if (joystick.getRawButton(EXIT_MAN_BUTTON))
            		state = saveState;
            	break;
            }
            
            if (joystick.getRawButton(RESET_BUTTON))
            	state = RESET;
        	
        	if (joystick.getRawButton(MAN_BUTTON))
        	{
        		saveState = state;
        		state = MANUAL;
        	}*/
        }
    }

    private void setLift(double setPoint){
    	rightLift.set(-setPoint / DISTANCE_PER_REV); // right is reflected
    	leftLift.set(setPoint / DISTANCE_PER_REV);
    }
    
    private boolean calibratedLift()
    {
    	boolean ready = false;
    	rightLift.disableSoftPositionLimits();
    	leftLift.disableSoftPositionLimits();
    	if (rightLift.getForwardLimitOK())
    	{
    		rightLift.setPercentMode();
    		rightLift.enableControl();
    		rightLift.set(25);
    	}
    	else
    	{
    		initRightLift();
    		ready = true;
    	}
    
    	if (leftLift.getReverseLimitOK())
    	{
    		leftLift.setPercentMode();
    		leftLift.enableControl();
    		leftLift.set(-25);
    	}
    	else
    	{
    		initRightLift();
    		if (ready)
    			return true;
    	}
    	return false;
    }
    
    /**
     * Runs during test mode
     */
    public void test() {
    }
}
