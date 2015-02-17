
package org.oastem.frc.rush;


import java.io.ObjectOutputStream.PutField;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.CameraServer;
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
	private static final int EMERGENCY_STOP = 11;
	private static final int EXIT_E_STOP = 6;

	// CONSTANTS
	private static final int PLAN_ENC_CPR = 497;
	private static final int DRIVE_ENC_CPR = 2048;
	private static final int LIFT_HEIGHT_LIMIT = 39;
	private static final double LIFT_GRAD_DISTANCE = .05;
	private static final double LIFT_BUFFER = .2;
	private static final double LIFT_DISTANCE_PER_REV = 6.5; // Thanks Mr. Miller!
	private static final double DRIVE_CIRCUMFERENCE = 6 * Math.PI;
	private static final double DRIVE_GEAR_RATIO = 1/8.45;
	private static final double RIGHT_LIFT_COMP = .5;

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
	private CameraServer camera;

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
	private static final int ABOVE_TOTE = 11;
	private static final int TOTE_DRIVE = 8;
	private static final int SECOND_TOTE = 22;
	private static final int ABOVE_BIN = 25;
	private static final int GRAB_BIN = 17;
	private static final int BIN_DRIVE = 26;
	private static final int BIN_TOTE = 36;

	// USERCONTROL STATES
	private static final int RESET = 0;
	private static final int READY_FOR_TOTE = 1;
	private static final int GRABBING_TOTE = 2;
	private static final int TOTE_GRABBED = 3;
	private static final int READY_FOR_NEXT = 4;
	private static final int READY_FOR_BIN = 5;
	private static final int GRABBING_BIN = 10;
	private static final int BIN_GRABBED = 6;
	private static final int READY_FOR_BIN_TOTE = 7;
	private static final int MANUAL = 8;
	private static final int COMPLETE_MANUAL = 9;
	private static final int E_STOP_STATE = 11;


	public void robotInit() {
		drive = DriveSystem.getInstance();
		drive.initializeEncoders(RIGHT_ENC_A, RIGHT_ENC_B, DRIVE_ENC_CPR);
		drive.initializeDrive(DRIVE_LEFT_FRONT_PORT, DRIVE_LEFT_BACK_PORT, DRIVE_RIGHT_FRONT_PORT, DRIVE_RIGHT_BACK_PORT);
		drive.setSafety(false);

		camera = CameraServer.getInstance();
		camera.setQuality(50);
		camera.startAutomaticCapture("cam0");

		power = new PowerDistributionPanel();
		power.clearStickyFaults();

		/*rightDriveFront = new Victor(DRIVE_RIGHT_FRONT_PORT);
        //rightDriveBack = new Victor(DRIVE_RIGHT_BACK_PORT);
        //leftDriveFront = new Victor(DRIVE_LEFT_FRONT_PORT);
        leftDriveBack = new Victor(DRIVE_LEFT_BACK_PORT);
		 */
		drive.setInvertedQuad();

		//rightEnc = new QuadratureEncoder(RIGHT_ENC_A, RIGHT_ENC_B, true, 4, DRIVE_ENC_CPR);
		//rightEnc.setDistancePerPulse(DRIVE_CIRCUMFERENCE); // * DRIVE_GEAR_RATIO);
		leftEnc = new QuadratureEncoder(LEFT_ENC_A, LEFT_ENC_B, 4, DRIVE_ENC_CPR);
		leftEnc.setDistancePerPulse(DRIVE_CIRCUMFERENCE); //* DRIVE_GEAR_RATIO);


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
		leftLift.setPositionMode(CANJaguar.kQuadEncoder, PLAN_ENC_CPR, 1500, 0.002, 1000);
		//leftLift.setPercentMode();
		leftLift.configForwardLimit(LIFT_HEIGHT_LIMIT/LIFT_DISTANCE_PER_REV);
		leftLift.configLimitMode(CANJaguar.LimitMode.SoftPositionLimits);
		leftLift.enableControl(0);
	}



	/*

	// JOY WANTS THESE TO BE INSTANCE VARIABLES???
	// YES SHE DOES

	private int state = 0;
	private int resetCount = 0;
	private long currTime = 0L;
	private long triggerStart = 0L;

	public void autonomous() {

		while(isAutonomous() && isEnabled()) {
    		//dash.putBoolean("Drive", drive.forward(6 * Math.PI));
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

		boolean slaveRight = true;
		boolean driveOnly = false;
		int state = 0;
		int saveState = 0;



		//rightEnc.reset();
		leftEnc.reset();


		while (isOperatorControl() && isEnabled()) {
			if (driveOnly) {
				slaveRight = false;
				state = E_STOP_STATE;
			}


			this.doArcadeDrive();
			//dash.putNumber("RIGHT DRIVE RAW", rightEnc.getRaw());
			dash.putNumber("LEFT DRIVE RAW", leftEnc.getRaw());
			//dash.putNumber("Right Drive: ", rightEnc.getDistance());
			dash.putNumber("Left Drive : ", leftEnc.getDistance());

			dash.putData("PDP: ", power);

			if (slaveRight && state != RESET)
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
				slaveRight = false;
				if (calibratedLift())
				{
					slaveRight = true;
					dash.putString("UP Button (6): ", "DNE");
					dash.putString("DOWN Button (7): ", "DNE");
					dash.putString("TOTE Button (5): ", "Gets ready to pick up TOTE");
					dash.putString("BIN Button (4): ", "Gets ready to pick up BIN");
					if (joyPayload.getRawButton(TOTE_BUTTON))
						state = READY_FOR_TOTE;
					if (joyPayload.getRawButton(BIN_BUTTON))
						state = READY_FOR_BIN;
				}
				
				else
				{	
					dash.putString("UP Button (6): ", "Calibrating...");
					dash.putString("DOWN Button (7): ", "Calibrating...");
					dash.putString("TOTE Button (5): ", "Calibrating...");
					dash.putString("BIN Button (4): ", "Calibrating...");
				}
				break;
			case READY_FOR_TOTE :
				dash.putString("UP Button (6): ", "Move hooks to over TOTE to pick it up");
				dash.putString("DOWN Button (7): ", "DNE");
				dash.putString("TOTE Button (5): ", "DNE");
				dash.putString("BIN Button (4): ", "Gets ready to pick up BIN");
				setLift(ABOVE_TOTE);
				if (joyPayload.getRawButton(LIFT_UP) && (Math.abs(rightLift.getPosition()*LIFT_DISTANCE_PER_REV - ABOVE_TOTE) < LIFT_BUFFER + RIGHT_LIFT_COMP) && (Math.abs(leftLift.getPosition()*LIFT_DISTANCE_PER_REV - ABOVE_TOTE) < LIFT_BUFFER))
					state = GRABBING_TOTE;
				if (joyPayload.getRawButton(BIN_BUTTON))
					state = READY_FOR_BIN;
				break;
			case READY_FOR_BIN :
				dash.putString("UP Button (6): ", "Picks up BIN to drive");
				dash.putString("DOWN Button (7): ", "DNE");
				dash.putString("TOTE Button (5): ", "Gets ready to pick up TOTE");
				dash.putString("BIN Button (4): ", "DNE");
				setLift(ABOVE_BIN);
				if (joyPayload.getRawButton(LIFT_UP) && (Math.abs(rightLift.getPosition()*LIFT_DISTANCE_PER_REV - ABOVE_BIN) < LIFT_BUFFER + RIGHT_LIFT_COMP) && (Math.abs(leftLift.getPosition()*LIFT_DISTANCE_PER_REV - ABOVE_BIN) < LIFT_BUFFER ))
					state = GRABBING_BIN;
				if (joyPayload.getRawButton(TOTE_BUTTON))
					state = READY_FOR_TOTE;
				break;
			case GRABBING_TOTE :
				dash.putString("UP Button (6): ", "Picks up TOTE to drive");
				dash.putString("DOWN Button (7): ", "DNE");
				dash.putString("TOTE Button (5): ", "Goes back to getting ready to pick up TOTE");
				dash.putString("BIN Button (4): ", "Gets ready to pick up BIN");
				setLift(BOTTOM);
				if (joyPayload.getRawButton(LIFT_UP) && (Math.abs(rightLift.getPosition()*LIFT_DISTANCE_PER_REV - BOTTOM) < LIFT_BUFFER + RIGHT_LIFT_COMP) && (Math.abs(leftLift.getPosition()*LIFT_DISTANCE_PER_REV - BOTTOM) < LIFT_BUFFER))
					state = TOTE_GRABBED;
				if (joyPayload.getRawButton(BIN_BUTTON))
					state = READY_FOR_BIN;
				if (joyPayload.getRawButton(TOTE_BUTTON))
					state = READY_FOR_TOTE;
				break;
			case GRABBING_BIN :
				setLift(GRAB_BIN);
				if (joyPayload.getRawButton(LIFT_UP) && (Math.abs(rightLift.getPosition()*LIFT_DISTANCE_PER_REV - GRAB_BIN) < LIFT_BUFFER + RIGHT_LIFT_COMP) && (Math.abs(leftLift.getPosition()*LIFT_DISTANCE_PER_REV - GRAB_BIN) < LIFT_BUFFER ))
					state = BIN_GRABBED;
				if (joyPayload.getRawButton(BIN_BUTTON))
					state = READY_FOR_BIN;
				if (joyPayload.getRawButton(TOTE_BUTTON))
					state = READY_FOR_TOTE;
				break;
			case TOTE_GRABBED :
				dash.putString("UP Button (6): ", "Gets ready to pick up another TOTE");
				dash.putString("DOWN Button (7): ", "Puts TOTE down");
				dash.putString("TOTE Button (5): ", "DNE");
				dash.putString("BIN Button (4): ", "DNE");
				setLift(TOTE_DRIVE);
				if ((Math.abs(rightLift.getPosition()*LIFT_DISTANCE_PER_REV - TOTE_DRIVE) < LIFT_BUFFER + RIGHT_LIFT_COMP) && (Math.abs(leftLift.getPosition()*LIFT_DISTANCE_PER_REV - TOTE_DRIVE) < LIFT_BUFFER))
				{
					if (joyPayload.getRawButton(LIFT_UP))
						state = READY_FOR_NEXT;
					else if (joyPayload.getRawButton(LIFT_DOWN))
						state = GRABBING_TOTE;
				}
				break;
			case BIN_GRABBED :
				dash.putString("UP Button (6): ", "Gets ready to pick up a TOTE under the BIN");
				dash.putString("DOWN Button (7): ", "Puts BIN down");
				dash.putString("TOTE Button (5): ", "Gets ready to pick up a TOTE under the BIN");
				dash.putString("BIN Button (4): ", "DNE");
				setLift(BIN_DRIVE);
				if ((Math.abs(rightLift.getPosition()*LIFT_DISTANCE_PER_REV - BIN_DRIVE) < LIFT_BUFFER + RIGHT_LIFT_COMP) && (Math.abs(leftLift.getPosition()*LIFT_DISTANCE_PER_REV - BIN_DRIVE) < LIFT_BUFFER ))
				{
					if ((joyPayload.getRawButton(LIFT_UP) || joyPayload.getRawButton(TOTE_BUTTON)))
						state = READY_FOR_BIN_TOTE;
					else if (joyPayload.getRawButton(LIFT_DOWN))
						state = GRABBING_BIN;
				}
				break;
			case READY_FOR_NEXT :
				dash.putString("UP Button (6): ", "Puts hooks under TOTE to grab another TOTE");
				dash.putString("DOWN Button (7): ", "Goes back to TOTE driving position");
				dash.putString("TOTE Button (5): ", "DNE");
				dash.putString("BIN Button (4): ", "DNE");
				setLift(SECOND_TOTE);
				if ((Math.abs(rightLift.getPosition()*LIFT_DISTANCE_PER_REV - SECOND_TOTE) < LIFT_BUFFER + RIGHT_LIFT_COMP) && (Math.abs(leftLift.getPosition()*LIFT_DISTANCE_PER_REV - SECOND_TOTE) < LIFT_BUFFER ))
				{
					if (joyPayload.getRawButton(LIFT_UP))
						state = GRABBING_TOTE;
					else if (joyPayload.getRawButton(LIFT_DOWN))
						state = TOTE_GRABBED;
				}
				break;
			case READY_FOR_BIN_TOTE :
				dash.putString("UP Button (6): ", "Puts hooks under TOTE to pick up TOTE and BIN");
				dash.putString("DOWN Button (7): ", "Puts BIN back into driving position");
				dash.putString("TOTE Button (5): ", "DNE");
				dash.putString("BIN Button (4): ", "DNE");
				setLift(BIN_TOTE);
				dash.putString("WHy", "isthisnotworking");
				if ((Math.abs(rightLift.getPosition()*LIFT_DISTANCE_PER_REV - BIN_TOTE) < LIFT_BUFFER + RIGHT_LIFT_COMP) && (Math.abs(leftLift.getPosition()*LIFT_DISTANCE_PER_REV - BIN_TOTE) < LIFT_BUFFER))
				{
					if (joyPayload.getRawButton(LIFT_UP))
						state = GRABBING_TOTE;
					else if (joyPayload.getRawButton(LIFT_DOWN))
						state = BIN_GRABBED;
				}
				break;
			case MANUAL :
				dash.putString("UP Button (6): ", "Moves hooks up by a small increment");
				dash.putString("DOWN Button (7): ", "Moves hooks down by a small increment");
				dash.putString("TOTE Button (5): ", "EMANUEL");
				dash.putString("BIN Button (4): ", "EMANUEL");
				dash.putString("EXIT MANUAL Button (9): ", "Exits the EMANUAL mode");
				if (joyPayload.getRawButton(EXIT_MAN_BUTTON))
					state = saveState;
				if (joyPayload.getRawButton(LIFT_UP))
					setLift(Math.abs(rightLift.get() * LIFT_DISTANCE_PER_REV) + LIFT_GRAD_DISTANCE);
				else if (joyPayload.getRawButton(LIFT_DOWN))
					setLift(Math.abs(rightLift.get() * LIFT_DISTANCE_PER_REV) - LIFT_GRAD_DISTANCE);
				break;
			case COMPLETE_MANUAL :
				dash.putString("UP Button (6): ", "COMPLETELY EMANUEL; move using JOYSTICK");
				dash.putString("DOWN Button (7): ", "COMPLETELY EMANUEL; move using JOYSTICK");
				dash.putString("TOTE Button (5): ", "COMPLETELY EMANUEL; move using JOYSTICK");
				dash.putString("BIN Button (4): ", "COMPLETELY EMANUEL; move using JOYSTICK");
				dash.putString("EXIT MANUAL Button (9): ", "Exits the COMPLETELY EMANUEL mode");
				rightLift.set(-joyPayload.getY());
				leftLift.set(-joyPayload.getY()*1.15);

				if (joyPayload.getRawButton(EXIT_MAN_BUTTON) || joyPayload.getRawButton(RESET_BUTTON)){
					state = RESET;
				}
				break;
			case E_STOP_STATE :
				dash.putString("UP Button (6): ", "Robot lift disabled");
				dash.putString("DOWN Button (7): ", "Robot lift disabled");
				dash.putString("TOTE Button (5): ", "Robot lift disabled");
				dash.putString("BIN Button (4): ", "Robot lift disabled");
				dash.putString("EXIT MANUAL Button (9): ", "Robot lift disabled");
				break;
			}


			if (joyPayload.getRawButton(RESET_BUTTON) && !driveOnly)
				state = RESET;

			if (joyPayload.getRawButton(MAN_BUTTON) && state != MANUAL && !driveOnly)
			{
				saveState = state;
				state = MANUAL;
			}

			if(joyPayload.getRawButton(LIFT_TOGGLE) && state != COMPLETE_MANUAL && !driveOnly)
			{
				slaveRight = false;
				disableLift();
				rightLift.setPercentMode();
				leftLift.setPercentMode();
				rightLift.enableControl();
				leftLift.enableControl();
				state = COMPLETE_MANUAL;
			}

			if (joyDrive.getRawButton(EMERGENCY_STOP) || joyPayload.getRawButton(EMERGENCY_STOP))
			{
				rightLift.disableControl();
				leftLift.disableControl();
				driveOnly = true;
				slaveRight = false;
				state = E_STOP_STATE;
			}
			
			if (joyDrive.getRawButton(EXIT_E_STOP) && state == E_STOP_STATE)
			{
				driveOnly = false;
				slaveRight = true;
				state = RESET;
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
		return Math.min(1.0, 0.5 - 0.3 * rawZ);
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
		leftLift.set(setPoint / LIFT_DISTANCE_PER_REV); 
		//rightLift.set(setPoint / LIFT_DISTANCE_PER_REV); // left is reflected
	}

	private void doSlave()
	{
		rightLift.set(leftLift.getPosition() - (RIGHT_LIFT_COMP/LIFT_DISTANCE_PER_REV));
	}


	private boolean calibratedLift()
	{
		boolean ready = false;
		disableLift();
		if (rightLift.getReverseLimitOK())
		{
			rightLift.setPercentMode();
			rightLift.enableControl();
			rightLift.set(-0.75);
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
			leftLift.set(-0.75);
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
