
package org.oastem.frc.rush;


import java.io.ObjectOutputStream.PutField;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GearTooth;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
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
    private static final int LIFT_TOGGLE = 1;
    private static final int BIN_BUTTON = 4;
    private static final int TOTE_BUTTON = 5;
    private static final int RESET_BUTTON = 3;
    private static final int MAN_BUTTON = 8;
    private static final int EXIT_MAN_BUTTON = 9;
    
    // CONSTANTS
    private static final int PLAN_ENC_CPR = 497;
    private static final int DRIVE_ENC_CPR = 2048;
    private static final int LIFT_HEIGHT_LIMIT = 39;
    private static final double LIFT_GRAD_DISTANCE = .05;
    private static final double LIFT_BUFFER = .5;
    private static final double LIFT_DISTANCE_PER_REV = 6.5; // Thanks Mr. Miller!
    private static final double DRIVE_CIRCUMFERENCE = 6 * Math.PI;
    private static final double DRIVE_GEAR_RATIO = 8.45/1;
    
    // instance variables
    private static double joyScale = 1.0;
    
    
    // DECLARING OBJECTS
    private DriveSystem drive;
    private Joystick joyDrive;
    private Joystick joyPayload;
    private Dashboard dash;
    private PowerDistributionPanel power;
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
	private static final int SECOND_TOTE = 16;
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
	private static final int COMPLETE_MANUAL = 9;
	

    public void robotInit() {
    	drive = new DriveSystem(4, 5, 3.0);
    	drive.initializeDrive(DRIVE_LEFT_FRONT_PORT, DRIVE_LEFT_BACK_PORT, DRIVE_RIGHT_FRONT_PORT, DRIVE_RIGHT_BACK_PORT);
        drive.setSafety(false);
        
        
        power = new PowerDistributionPanel();
        
    	/*rightDriveFront = new Victor(DRIVE_RIGHT_FRONT_PORT);
        //rightDriveBack = new Victor(DRIVE_RIGHT_BACK_PORT);
        //leftDriveFront = new Victor(DRIVE_LEFT_FRONT_PORT);
        leftDriveBack = new Victor(DRIVE_LEFT_BACK_PORT);
        */
        drive.setInvertedQuad();
        
        rightEnc = new QuadratureEncoder(RIGHT_ENC_A, RIGHT_ENC_B, RIGHT_ENC_I, true, DRIVE_ENC_CPR);
        rightEnc.setDistancePerPulse(DRIVE_CIRCUMFERENCE * DRIVE_GEAR_RATIO);
        leftEnc = new QuadratureEncoder(LEFT_ENC_A, LEFT_ENC_B, LEFT_ENC_I, DRIVE_ENC_CPR);
        leftEnc.setDistancePerPulse(DRIVE_CIRCUMFERENCE * DRIVE_GEAR_RATIO);
        
        
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
        
        joyDrive = new Joystick(0);
        joyPayload = new Joystick(1);
        
        dash = new Dashboard();
        System.out.println("Robot Initialized");
    }
    
    private void initRightLift()
    {
    	rightLift.free();
        rightLift = new CANJaguar(RIGHT_LIFT_PORT);
    	rightLift.setPositionMode(CANJaguar.kQuadEncoder, PLAN_ENC_CPR, 1000, 0.002, 1000);
    	//rightLift.setPercentMode();
        rightLift.configForwardLimit(LIFT_HEIGHT_LIMIT/LIFT_DISTANCE_PER_REV);
    	rightLift.configLimitMode(CANJaguar.LimitMode.SoftPositionLimits);
        rightLift.enableControl(0);
        
    }
    private void initLeftLift()
    {
    	leftLift.free();
        leftLift = new CANJaguar(LEFT_LIFT_PORT);
        leftLift.setPositionMode(CANJaguar.kQuadEncoder, PLAN_ENC_CPR, 1000, 0.002, 1000);
    	//leftLift.setPercentMode();
        leftLift.configForwardLimit(LIFT_HEIGHT_LIMIT/LIFT_DISTANCE_PER_REV);
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
    	
    	
    	final boolean TESTING_DRIVE = true;
    	
    	rightEnc.reset();
    	leftEnc.reset();
    	
    	//drive.forward(10.0);
    	
        while (isOperatorControl() && isEnabled()) {
        	if (TESTING_DRIVE) {
        		slaveLeft = false;
        		state = 25;
        	}
        	
        	doArcadeDrive();
            //drive.arcadeDrive(joyDrive.getY()*joyDrive.getZ(), joyDrive.getX());
            dash.putNumber("Right Drive: ", rightEnc.getDistance());
            dash.putNumber("Left Drive : ", leftEnc.getDistance());
            
            dash.putData("PDP: ", power);

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
            		if (joyPayload.getRawButton(TOTE_BUTTON))
            			state = READY_FOR_TOTE;
            		if (joyPayload.getRawButton(BIN_BUTTON))
            			state = READY_FOR_BIN;
            	}
            	break;
            case READY_FOR_TOTE :
            	dash.putString("Status: ", "in READY_FOR_TOTE");
            	setLift(ABOVE_TOTE);
            	if (joyPayload.getRawButton(LIFT_UP) && (Math.abs(rightLift.getPosition()*LIFT_DISTANCE_PER_REV - ABOVE_TOTE) < LIFT_BUFFER))// && leftLift.getPosition() == ABOVE_TOTE)
            		state = GRABBING_TOTE;
            	if (joyPayload.getRawButton(BIN_BUTTON))
        			state = READY_FOR_BIN;
            	break;
            case READY_FOR_BIN :
            	dash.putString("Status: ", "in READY_FOR_BIN");
            	setLift(GRAB_BIN);
            	if (joyPayload.getRawButton(LIFT_UP) && (Math.abs(rightLift.getPosition()*LIFT_DISTANCE_PER_REV - GRAB_BIN) < LIFT_BUFFER))// && //leftLift.getPosition() == GRAB_BIN)
            		state = BIN_GRABBED;
            	if (joyPayload.getRawButton(TOTE_BUTTON))
        			state = READY_FOR_TOTE;
            	break;
            case GRABBING_TOTE :
            	dash.putString("Status: ", "in GRABBING_TOTE");
            	setLift(BOTTOM);
            	if (joyPayload.getRawButton(LIFT_UP) && (Math.abs(rightLift.getPosition()*LIFT_DISTANCE_PER_REV - BOTTOM) < LIFT_BUFFER))// && leftLift.getPosition() == BOTTOM)
            		state = TOTE_GRABBED;
            	if (joyPayload.getRawButton(BIN_BUTTON))
        			state = READY_FOR_BIN;
            	if (joyPayload.getRawButton(TOTE_BUTTON))
        			state = READY_FOR_TOTE;
            	break;
            case TOTE_GRABBED :
            	dash.putString("Status: ", "in TOTE_GRABBED");
            	setLift(TOTE_DRIVE);
            	if (Math.abs(rightLift.getPosition()*LIFT_DISTANCE_PER_REV - TOTE_DRIVE) < LIFT_BUFFER)// && leftLift.getPosition() == TOTE_DRIVE)
            	{
            		if (joyPayload.getRawButton(LIFT_UP))
            			state = READY_FOR_NEXT;
            		else if (joyPayload.getRawButton(LIFT_DOWN))
            			state = GRABBING_TOTE;
            	}
            	break;
            case BIN_GRABBED :
            	dash.putString("Status: ", "in BIN_GRABBED");
            	setLift(BIN_DRIVE);
            	if (Math.abs(rightLift.getPosition()*LIFT_DISTANCE_PER_REV - BIN_DRIVE) < LIFT_BUFFER)// && leftLift.getPosition() == BIN_DRIVE)
            	{
            		if ((joyPayload.getRawButton(LIFT_UP) || joyPayload.getRawButton(TOTE_BUTTON)))
            			state = READY_FOR_BIN_TOTE;
            		else if (joyPayload.getRawButton(LIFT_DOWN))
            			state = READY_FOR_BIN;
            	}
            	break;
            case READY_FOR_NEXT :
            	dash.putString("Status: ", "in READY_FOR_NEXT");
            	setLift(SECOND_TOTE);
            	if(Math.abs(rightLift.getPosition()*LIFT_DISTANCE_PER_REV - SECOND_TOTE) < LIFT_BUFFER)// && leftLift.getPosition() == SECOND_TOTE)
            	{
            		if (joyPayload.getRawButton(LIFT_UP))
            			state = GRABBING_TOTE;
            		else if (joyPayload.getRawButton(LIFT_DOWN))
            			state = TOTE_GRABBED;
            	}
            	break;
            case READY_FOR_BIN_TOTE :
            	dash.putString("Status: ", "in READY_FOR_BIN_TOTE");
            	setLift(BIN_TOTE);
            	if(Math.abs(rightLift.getPosition()*LIFT_DISTANCE_PER_REV - BIN_TOTE) < LIFT_BUFFER)// && leftLift.getPosition() == BIN_TOTE)
            	{
            		if (joyPayload.getRawButton(LIFT_UP))
            			state = GRABBING_TOTE;
            		else if (joyPayload.getRawButton(LIFT_DOWN))
            			state = BIN_GRABBED;
            	}
            	if (joyPayload.getRawButton(LIFT_UP))
            		break;
            case MANUAL :
            	dash.putString("Status: ", "EMANUEL");
            	state = saveState;
            	if (joyPayload.getRawButton(EXIT_MAN_BUTTON))
            		setLift(Math.abs(rightLift.get() * LIFT_DISTANCE_PER_REV) + LIFT_GRAD_DISTANCE);
            	else if (joyPayload.getRawButton(LIFT_DOWN))
            		setLift(Math.abs(rightLift.get() * LIFT_DISTANCE_PER_REV) - LIFT_GRAD_DISTANCE);
        		break;
            case COMPLETE_MANUAL :
            	dash.putString("Status: ", "COMPLETELY_EMANUEL");
        		rightLift.set(-joyPayload.getY()*joyPayload.getZ());
        		leftLift.set(-joyPayload.getY()*joyPayload.getZ());
        		
        		if (joyPayload.getRawButton(EXIT_MAN_BUTTON)){
        			slaveLeft = true;
        			state = saveState;
        			rightLift.free();
        			leftLift.free();
            		rightLift = new CANJaguar(RIGHT_LIFT_PORT);
            		leftLift = new CANJaguar(LEFT_LIFT_PORT);
            		initRightLift();
            		initLeftLift();
            	}
        		break;
            case 25 :
            	dash.putString("Status: ", "Lift disabled");
            	break;
            }
            
            
            if (joyPayload.getRawButton(RESET_BUTTON))
            	state = RESET;
        	
        	if (joyPayload.getRawButton(MAN_BUTTON) && state != MANUAL)
        	{
        		saveState = state;
        		state = MANUAL;
        	}
        	
        	if(joyPayload.getRawButton(LIFT_TOGGLE) && state != COMPLETE_MANUAL)
        	{
        		saveState = state;
            	disableLift();
            	slaveLeft = false;
            	rightLift.setPercentMode();
            	leftLift.setPercentMode();
        		rightLift.enableControl();
        		leftLift.enableControl();
        		state = COMPLETE_MANUAL;
        		
        	}
        	
        	
        	
        }
    }
    
    private void doArcadeDrive() {
        double leftMove = 0.0;
        double rightMove = 0.0;
        double zone = 0.04;

        joyScale = scaleZ(joyDrive.getZ());

        double x = joyDrive.getX();
        double y = joyDrive.getY() * -1;

        if (Math.abs(y) > zone) {
            leftMove = y;
            rightMove = y;
        }

        if (Math.abs(x) > zone) {
            leftMove = correct(leftMove + x);
            rightMove = correct(rightMove - x);
        }

        leftMove *= joyScale * -1;
        rightMove *= joyScale * -1;
        
        drive.tankDrive(leftMove, rightMove);
    }
    
    private double scaleZ(double rawZ) {
        return Math.min(1.0, 0.5 - 0.5 * rawZ);
    }
    
    private double correct(double val) {
        if (val > 1.0) {
            return 1.0;
        }
        if (val < -1.0) {
            return -1.0;
        }
        return val;
    }

    private void setLift(double setPoint){
    	rightLift.set(setPoint / LIFT_DISTANCE_PER_REV); 
    	//leftLift.set(setPoint / DISTANCE_PER_REV); // left is reflected
    }
    
    private void doSlave()
    {
    	leftLift.set(rightLift.getPosition()-(LIFT_BUFFER/LIFT_DISTANCE_PER_REV));
    }
    
    private boolean calibratedLift()
    {
    	boolean ready = false;
    	disableLift();
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

	private void disableLift() {
		rightLift.disableSoftPositionLimits();
    	leftLift.disableSoftPositionLimits();
    	leftLift.free();
    	rightLift.free();
    	leftLift = new CANJaguar(LEFT_LIFT_PORT);
    	rightLift = new CANJaguar(RIGHT_LIFT_PORT);
	}
    
    private void moveDistance(double distance){
    	
    }
    
    /**
     * Runs during test mode
     */
    public void test() {
    }
}
