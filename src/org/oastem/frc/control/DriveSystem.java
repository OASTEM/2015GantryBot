/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.oastem.frc.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;

import org.oastem.frc.sensor.*;

import java.util.Hashtable;

/**
 *
 * @author KTOmega
 */
public class DriveSystem {
    private static DriveSystem instance;
    private RobotDrive drive;
    private Victor[] raw;
    private double[] speed;
    private boolean hasSecondary = false;
    private RobotDrive drive2;
    
	private long currTime;
	private long thisTime;
	//private double[] speed;
	//private int[] locs;
	//private Accelerator[] acceleration;
	//private int locCount = 4;
	private QuadratureEncoder enc;
	
    protected DriveSystem() {
        raw = new Victor[12];
    }
    
	public DriveSystem(int channelA, int channelB, double pulses){
		currTime = System.currentTimeMillis();
		thisTime = currTime;
		//acceleration = new Accelerator[12];
		//enc = new QuadratureEncoder(channelA, channelB, pulses);
		speed = new double[12];
		//locs = new int[12];
		raw = new Victor[12];
		getInstance();
	}
	
    
    public DriveSystem getInstance() {
        if (instance == null) {
            instance = new DriveSystem();
        }
        
        return instance;
    }
    
    public void initializeDrive(int leftFront, int leftRear, int rightFront, int rightRear) {
        drive = new RobotDrive(leftFront, leftRear, rightFront, rightRear);
    }
    
    public void initializeDrive(int left, int right){
        drive = new RobotDrive(left, right);
    }
    
    public void setDrive(RobotDrive rd) {
        drive = rd;
    }
    
    public void initializeSecondaryDrive(int l2, int r2) {
        drive2 = new RobotDrive(l2, r2);
        hasSecondary = true;
    }
    
    public void setSecondaryDriver(RobotDrive rd) {
        drive2 = rd;
        hasSecondary = true;
    }
    
    public void arcadeDrive(Joystick joystick){
        drive.arcadeDrive(joystick);
    }
    
    public void arcadeDrive(double forward, double turn) {
        drive.arcadeDrive(forward, turn);
        if (hasSecondary) drive2.arcadeDrive(forward, turn);
    }
    
    public void tankDrive(double x, double y) {
        drive.tankDrive(x, y);
        if (hasSecondary) drive2.tankDrive(x, y);
    }
    
    public void mecanumDrive(double x, double y, double turn, double gyro)
    {
        drive.mecanumDrive_Cartesian(x, y, turn, gyro);
        if (hasSecondary) drive2.mecanumDrive_Cartesian(x, y, turn, gyro);
    }
    
    public void addVictor(int port) {
        raw[port] = new Victor(port);
    }
    
    public void set(int vic, double power) {
        raw[vic].set(power);
    }
    
    public double getPwm(int vic) {
        return raw[vic].get();
    }
    
    public Victor getVictor(int vic) {
        return raw[vic];
    }
    
    public void setSafety(boolean b){
        drive.setSafetyEnabled(false);
        if (hasSecondary) drive2.setSafetyEnabled(false);
    }

	public boolean forward(double distance) {
		enc.reset();
		if (enc.getDistance() < distance) {
			drive.arcadeDrive(0.5, 0.5);
			if (hasSecondary) drive2.arcadeDrive(0.5, 0.5);
			return false;
		} else {
			return true;
		}
	}

	public boolean reverse(double distance) {
		enc.reset();
		if (Math.abs(enc.getDistance()) < distance) {
			drive.arcadeDrive(-0.5, -0.5);
			if (hasSecondary) drive2.arcadeDrive(-0.5, -0.5);
			return false;
		} else {
			return true;
		}
	}
    
    public void setInvertedDouble()
    {
    	drive.setInvertedMotor(MotorType.kRearLeft, true);
    	drive.setInvertedMotor(MotorType.kRearRight, true);
    }
    
    public void setInvertedQuad()
    {
    	drive.setInvertedMotor(MotorType.kFrontLeft, true);
    	drive.setInvertedMotor(MotorType.kFrontRight, true);
    	drive.setInvertedMotor(MotorType.kRearLeft, true);
    	drive.setInvertedMotor(MotorType.kRearRight, true);
    }
}
