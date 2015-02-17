package org.oastem.frc.control;

import edu.wpi.first.wpilibj.Joystick;
import org.oastem.frc.sensor.*;


/**
 * @author ThePotatoGuy
 * @author joyhsu0504
 */

public class DriveSystemAccel extends DriveSystem {
	private static DriveSystemAccel instance;
	
	private long currTime;
	private long thisTime;
	private double[] speed;
	private int[] locs;
	private Accelerator[] acceleration;
	private int locCount = 4;
	
	protected DriveSystemAccel(){
		super();
		currTime = System.currentTimeMillis();
		thisTime = currTime;
		acceleration = new Accelerator[NUM_ITEMS];
		speed = new double[NUM_ITEMS];
		locs = new int[NUM_ITEMS];
	}
	
	public static DriveSystemAccel getInstance() {
		if (instance == null) {
			instance = new DriveSystemAccel();
		}
		
		return instance;
	}
	
	public void initializeEncoders(int channelA, int channelB, double pulses) {
		super.initializeEncoders(channelA, channelB, pulses);
	}
	
    public void initializeDrive(int leftFront, int leftRear, int rightFront, int rightRear) {
		locs[0] = leftFront;
		//locs[1] = leftRear;
		locs[2] = rightFront;
		//locs[3] = rightRear;
		acceleration[leftFront] = new Accelerator();
		//acceleration[leftRear] = new Accelerator();
		acceleration[rightFront] = new Accelerator();
		//acceleration[rightRear] = new Accelerator();
        super.initializeDrive(leftFront, leftRear, rightFront, rightRear);
    }
    
    public void initializeSecondaryDrive(int l2, int r2) {
		locs[locCount++] = l2;
		acceleration[l2] = new Accelerator();
		locs[locCount++] = r2;
		acceleration[r2] = new Accelerator();
        super.initializeSecondaryDrive(l2, r2);
    }
    
    public void addVictor(int port) {
		locs[locCount++] = port;
		//acceleration[port] = new Accelerator();
        super.addVictor(port);
    }
    
    public void set(int vic, double power) {
		speed[locs[vic]] = power;
        super.set(vic,power);
    }
    
    public void tankDrive(double x, double y) {
		x = acceleration[locs[0]].accelerateValue(x);
		y = acceleration[locs[2]].accelerateValue(y);
		super.tankDrive(x,y);
    }

	public boolean drive(double distance, double x, double y) {
		enc.reset();
		x = acceleration[locs[0]].accelerateValue(0.5);
		y = acceleration[locs[2]].accelerateValue(0.5);
		if (enc.getDistance() < distance) {
			super.arcadeDrive(x, y);
			return false;
		} else {
			return true;
		}
	}

	public boolean reverse(double distance, double x, double y) {
		enc.reset();
		x = acceleration[locs[0]].accelerateValue(-0.5);
		y = acceleration[locs[2]].accelerateValue(-0.5);
		if (Math.abs(enc.getDistance()) < distance) {
			super.arcadeDrive(x, y);
			return false;
		} else {
			return true;
		}
	}
	

    public double getAccelSpeed(int port){
		return acceleration[locs[port]].getSpeed();
	}
}

