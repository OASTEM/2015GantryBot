package org.oastem.frc.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;

import org.oastem.frc.sensor.*;

import java.util.Hashtable;

/**
 * Class to control the entirety of the drive train of an FRC robot.
 * Originally designed by Kevin Tran for FRC 4079 in 2013: Ultimate Ascent.
 * Modified by future years of FRC 4079.
 * 
 * @author KTOmega
 */
public class DriveSystem {
    // Constants.
    protected static final int NUM_ITEMS = 12;
    protected static final double DISTANCE_PER_REVOLUTION = 6 * Math.PI; // FOR DEFAULT DRIVE WHEELS 
    
    // Singleton design pattern: instance of this class.
    // Only one drive system is allowed per robot - 
    // if any class needs it, it can call the getInstance()
    // method to use it.
    private static DriveSystem instance;
    
    private RobotDrive drive;
    private Victor[] raw;
    private boolean hasSecondary = false;
    private RobotDrive drive2;
    
    protected QuadratureEncoder encRight;
    protected QuadratureEncoder encLeft;
    
    protected DriveSystem() {
        raw = new Victor[NUM_ITEMS];
    }
    
    public static DriveSystem getInstance() {
        if (instance == null) {
            instance = new DriveSystem();
        }
        
        return instance;
    }
    
    public void initializeEncoders(int rightChannelA, int rightChannelB, boolean rightReflected,
    								int leftChannelA, int leftChannelB, boolean leftReflected, double pulsesPerRev) {
        encRight = new QuadratureEncoder(rightChannelA, rightChannelB, rightReflected, 4, pulsesPerRev);
        encLeft = new QuadratureEncoder(leftChannelA, leftChannelB, leftReflected, 4, pulsesPerRev);
        encRight.setDistancePerPulse(DISTANCE_PER_REVOLUTION);
        encLeft.setDistancePerPulse(DISTANCE_PER_REVOLUTION);
        encRight.reset();
        encLeft.reset();
    }
    
    public double getRightEnc()
    {
    	return encRight.getDistance();
    }
    
    public double getLeftEnc()
    {
    	return encLeft.getDistance();
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
    
    public void mecanumDrive(double x, double y, double turn, double gyro) {
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
    
    public void setSafety(boolean b) {
        drive.setSafetyEnabled(false);
        if (hasSecondary) drive2.setSafetyEnabled(false);
    }

    public void resetEncoders()
    {
    	encRight.reset();
    	encLeft.reset();
    }
    
    public boolean reverse(double distance) {
        if (Math.abs(encRight.getDistance()) < distance) {
            drive.arcadeDrive(0.5, 0.5);
            if (hasSecondary) drive2.arcadeDrive(0.5, 0.5);
            return false;
        } else {
            drive.arcadeDrive(0, 0);
            return true;
        }
    }

    public boolean forward(double distance) {
        if (encRight.getDistance() < distance) {
            drive.arcadeDrive(-0.35, -0.35);
            if (hasSecondary) drive2.arcadeDrive(-0.5, -0.5);
            return false;
        } else {
            drive.arcadeDrive(0, 0);
            return true;
        }
    }
    
    public void setInvertedDouble() {
        drive.setInvertedMotor(MotorType.kRearLeft, true);
        drive.setInvertedMotor(MotorType.kRearRight, true);
    }
    
    public void setInvertedQuad() {
        drive.setInvertedMotor(MotorType.kFrontLeft, true);
        drive.setInvertedMotor(MotorType.kFrontRight, true);
        drive.setInvertedMotor(MotorType.kRearLeft, true);
        drive.setInvertedMotor(MotorType.kRearRight, true);
    }
}
