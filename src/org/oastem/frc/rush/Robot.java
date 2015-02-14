
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
    
    // MOTOR PORTS
    private static final int DRIVE_RIGHT_FRONT_PORT = 2;
    private static final int DRIVE_RIGHT_BACK_PORT = 3;
    private static final int DRIVE_LEFT_FRONT_PORT = 0;
    private static final int DRIVE_LEFT_BACK_PORT = 1;
    
    // SENSOR PORTS
    private static final int RIGHT_LIFT_PORT = 4;
    private static final int LEFT_LIFT_PORT = 3;
    
    private static final int RIGHT_ENC_I = 4;
    private static final int RIGHT_ENC_A = 5;
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
    private static final int BIN_BUTTON = 4;
    private static final int TOTE_BUTTON = 5;
    private static final int RESET_BUTTON = 3;
    private static final int MAN_BUTTON = 10;
    private static final int EXIT_MAN_BUTTON = 11;
    
    // CONSTANTS
    private static final int PLAN_ENC_CPR = 497;
    private static final int LIFT_HEIGHT_LIMIT = 39;
    private static final double LIFT_GRAD_DISTANCE = .05;
    private static final double LIFT_BUFFER = .5;
    private static final double DISTANCE_PER_REV = 6.5; // Thanks Mr. Miller!
    
    
    // DECLARING OBJECTS
    private DriveSystem drive;
    private Joystick joystick;
    private Dashboard dash;
    private QuadratureEncoder rightEnc;
    private QuadratureEncoder leftEnc;
    
    // AUTONOMOUS STATES
    public static final int START = 0;
	public static final int GOTO_TOTE = 1;
	public static final int UPLIFT = 2;
	public static final int REDO = 3;
	public static final int MOVETO_AUTO = 4;
	public static final int RELEASE = 5;
	public static final int READY = 6;
    
	
	// CLAW HEIGHTS
	private static final int BOTTOM = 0;
	private static final int ABOVE_TOTE = 10;
	private static final int TOTE_DRIVE = 6;
	private static final int SECOND_TOTE = 15;
	private static final int GRAB_BIN = 12;
	private static final int BIN_DRIVE = 18;
	private static final int BIN_TOTE = 29;
	
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
        //drive.initializeDrive(DRIVE_LEFT_FRONT_PORT, DRIVE_LEFT_BACK_PORT, DRIVE_RIGHT_FRONT_PORT, DRIVE_RIGHT_BACK_PORT);
    	drive = new DriveSystem(4, 5, 3.0);
        drive.initializeDrive(DRIVE_LEFT_BACK_PORT, DRIVE_RIGHT_BACK_PORT);
        drive.setSafety(false);
        
        
    	/*rightDriveFront = new Victor(DRIVE_RIGHT_FRONT_PORT);
        //rightDriveBack = new Victor(DRIVE_RIGHT_BACK_PORT);
        //leftDriveFront = new Victor(DRIVE_LEFT_FRONT_PORT);
        leftDriveBack = new Victor(DRIVE_LEFT_BACK_PORT);
        */
        drive.setInvertedDouble();
        
        rightEnc = new QuadratureEncoder(RIGHT_ENC_A, RIGHT_ENC_B, RIGHT_ENC_I);
        leftEnc = new QuadratureEncoder(LEFT_ENC_A, LEFT_ENC_B, LEFT_ENC_I);
        
        
    	if (rightLift != null)
    	{
    		System.out.println("Right freed");
    		rightLift.free();	
    	}
    	if (leftLift != null)
    	{
    		System.out.println("Left freed");
    		leftLift.free();
    	}
    	rightLift = new CANJaguar(RIGHT_LIFT_PORT);
        leftLift = new CANJaguar(LEFT_LIFT_PORT);
        
        initRightLift();
        initLeftLift();
        
        joystick = new Joystick(0);
        
        dash = new Dashboard();
        System.out.println("Robot Initialized");
    }
    
    private void initRightLift()
    {
    	rightLift.free();
        rightLift = new CANJaguar(RIGHT_LIFT_PORT);
    	rightLift.setPositionMode(CANJaguar.kQuadEncoder, PLAN_ENC_CPR, 1000, 0.002, 1000);
    	//rightLift.setPercentMode();
        rightLift.configForwardLimit(LIFT_HEIGHT_LIMIT/DISTANCE_PER_REV);
    	rightLift.configLimitMode(CANJaguar.LimitMode.SoftPositionLimits);
        rightLift.enableControl(0);
        
    }
    private void initLeftLift()
    {
    	leftLift.free();
        leftLift = new CANJaguar(LEFT_LIFT_PORT);
        leftLift.setPositionMode(CANJaguar.kQuadEncoder, PLAN_ENC_CPR, 1000, 0.002, 1000);
    	//leftLift.setPercentMode();
        leftLift.configForwardLimit(LIFT_HEIGHT_LIMIT/DISTANCE_PER_REV);
        leftLift.configLimitMode(CANJaguar.LimitMode.SoftPositionLimits);
        leftLift.enableControl(0);
    }

	
	/*
	
	// JOY WANTS THESE TO BE INSTANCE VARIABLES???
	private int state = 0;
	private int resetCount = 0;
	private long currTime = 0L;
	private long triggerStart = 0L;
	
	public void autonomous() {
		
		while(isAutonomous() && isEnabled()) {
			//imageProcessing();
			currTime = System.currentTimeMillis();
			joytonomousStates(currTime); 
		}
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
						state = REDO; //count attempts
						resetCount++;
					}
				} else {
					triggerStart = currTime;
					state = MOVETO_AUTO;
				}
				break;
			case REDO:
				if(redo(currTime, triggerStart)) {
					triggerStart = currTime;
					state = UPLIFT;
				}
				break;
			case MOVETO_AUTO:
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
		if(!drive.drive(10) && currTime - triggerStart <= 1500L) { //lol this isn't a method but should return T/F 
			return false;
		} else {
			return true;
		}
	}
	
	private boolean hookUp(long currTime, long triggerStart) {
		drive.upToHook(); //lol I wish this were already a method
		if(!robot.checkHooked()) { //however long it takes to hook the tote
			return false;
		} else {
			return true;
		}
	}
	
	private boolean redo(long currTime, long triggerStart) {
		robot.downToUnhook();
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
		robot.downToUnhook();
		if(robot.checkHooked() && currTime - triggerStart <= 1500L) {
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
    	boolean calibrated = true;
    	boolean slaveLeft = true;
    	int state = 0;
    	int saveState = 0;
    	
    	
    	rightEnc.reset();
    	leftEnc.reset();
    	
    	drive.forward(10.0);
    	
        while (isOperatorControl() && isEnabled()) {
            //drive.arcadeDrive(joystick);
            dash.putNumber("Right Drive: ", rightEnc.getRaw());
            dash.putNumber("Left Drive : ", leftEnc.getRaw());
            

        	if (slaveLeft)
        		doSlave();
        	
        	
        	dash.putBoolean("LEFT: Limit Switch forward", leftLift.getForwardLimitOK());
        	dash.putBoolean("LEFT: Limit Switch reverse", leftLift.getReverseLimitOK());
        	dash.putBoolean("RIGHT: Limit Switch forward", rightLift.getForwardLimitOK());
        	dash.putBoolean("RIGHT: Limit Switch reverse", rightLift.getReverseLimitOK());
        	/////////////*/
            
        	
        	dash.putNumber("LEFT Lift", leftLift.getPosition());
        	dash.putNumber("RIGHT Lift", (rightLift.getPosition()));
        	
        	
        	
        	
        	
            switch(state)
            {
            case RESET :
        		slaveLeft = false;
        		dash.putString("Status: ", "Calibrating...");
            	if (calibratedLift())
            	{
                	slaveLeft = true;
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
            	if (joystick.getRawButton(LIFT_UP) && (Math.abs(rightLift.getPosition()*DISTANCE_PER_REV - ABOVE_TOTE) < LIFT_BUFFER))// && leftLift.getPosition() == ABOVE_TOTE)
            		state = GRABBING_TOTE;
            	if (joystick.getRawButton(BIN_BUTTON))
        			state = READY_FOR_BIN;
            	break;
            case READY_FOR_BIN :
            	dash.putString("Status: ", "in READY_FOR_BIN");
            	setLift(GRAB_BIN);
            	if (joystick.getRawButton(LIFT_UP) && (Math.abs(rightLift.getPosition()*DISTANCE_PER_REV - GRAB_BIN) < LIFT_BUFFER))// && //leftLift.getPosition() == GRAB_BIN)
            		state = BIN_GRABBED;
            	if (joystick.getRawButton(TOTE_BUTTON))
        			state = READY_FOR_TOTE;
            	break;
            case GRABBING_TOTE :
            	dash.putString("Status: ", "in GRABBING_TOTE");
            	setLift(BOTTOM);
            	if (joystick.getRawButton(LIFT_UP) && (Math.abs(rightLift.getPosition()*DISTANCE_PER_REV - BOTTOM) < LIFT_BUFFER))// && leftLift.getPosition() == BOTTOM)
            		state = TOTE_GRABBED;
            	if (joystick.getRawButton(BIN_BUTTON))
        			state = READY_FOR_BIN;
            	if (joystick.getRawButton(TOTE_BUTTON))
        			state = READY_FOR_TOTE;
            	break;
            case TOTE_GRABBED :
            	dash.putString("Status: ", "in TOTE_GRABBED");
            	setLift(TOTE_DRIVE);
            	if (Math.abs(rightLift.getPosition()*DISTANCE_PER_REV - TOTE_DRIVE) < LIFT_BUFFER)// && leftLift.getPosition() == TOTE_DRIVE)
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
            	if (Math.abs(rightLift.getPosition()*DISTANCE_PER_REV - BIN_DRIVE) < LIFT_BUFFER)// && leftLift.getPosition() == BIN_DRIVE)
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
            	if(Math.abs(rightLift.getPosition()*DISTANCE_PER_REV - SECOND_TOTE) < LIFT_BUFFER)// && leftLift.getPosition() == SECOND_TOTE)
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
            	if(Math.abs(rightLift.getPosition()*DISTANCE_PER_REV - BIN_TOTE) < LIFT_BUFFER)// && leftLift.getPosition() == BIN_TOTE)
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
            		setLift(Math.abs(rightLift.get() * DISTANCE_PER_REV) + LIFT_GRAD_DISTANCE);
            	else if (joystick.getRawButton(LIFT_DOWN))
            		setLift(Math.abs(rightLift.get() * DISTANCE_PER_REV) - LIFT_GRAD_DISTANCE);
            		
            	if (joystick.getRawButton(EXIT_MAN_BUTTON))
            		state = saveState;
            	break;
            }
            
            if (joystick.getRawButton(RESET_BUTTON))
            	state = RESET;
        	
        	if (joystick.getRawButton(MAN_BUTTON) && state != MANUAL)
        	{
        		saveState = state;
        		state = MANUAL;
        	}
        }
    }

    private void setLift(double setPoint){
    	rightLift.set(setPoint / DISTANCE_PER_REV); 
    	//leftLift.set(setPoint / DISTANCE_PER_REV); // left is reflected
    }
    
    private void doSlave()
    {
    	leftLift.set(rightLift.getPosition()-(0.5/DISTANCE_PER_REV));
    }
    
    private boolean calibratedLift()
    {
    	boolean ready = false;
    	rightLift.disableSoftPositionLimits();
    	leftLift.disableSoftPositionLimits();
    	leftLift.free();
    	rightLift.free();
    	leftLift = new CANJaguar(LEFT_LIFT_PORT);
    	rightLift = new CANJaguar(RIGHT_LIFT_PORT);
    	if (rightLift.getReverseLimitOK())
    	{
    		rightLift.setPercentMode();
    		rightLift.enableControl();
    		rightLift.set(-1);
    	}
    	else
    	{
    		rightLift.free();
    		rightLift = new CANJaguar(RIGHT_LIFT_PORT);
    		initRightLift();
    		ready = true;
    	}
    
    	if (leftLift.getReverseLimitOK())
    	{
    		leftLift.setPercentMode();
    		leftLift.enableControl();
    		leftLift.set(-1);
    	}
    	else
    	{
    		leftLift.free();
    		leftLift = new CANJaguar(LEFT_LIFT_PORT);
    		initLeftLift();
    		if (ready)
    			return true;
    	}
    	return false;
    }
    
    private void moveDistance(double distance){
    	
    }
    
    /**
     * Runs during test mode
     */
    public void test() {
    }
}
