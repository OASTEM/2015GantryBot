/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.oastem.frc.sensor;

import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.I2C.Port;

/**
 * Custom class built that encloses the Accelerometer class in order to 
 * accomodate for drifting/out of range values.
 * 
 * @author KTOmega
 */
public class ADXL345Accelerometer {
    private ADXL345_I2C accel;
    private double[] initialReadings;
    private double reactValue;
    
    public ADXL345Accelerometer(Port port) {
        this(port, 0.02);
    }
    
    public ADXL345Accelerometer(Port port, double react) {
        accel = new ADXL345_I2C(port, ADXL345_I2C.Range.k8G);//ADXL345_I2C.DataFormat_Range.k8G);
        initialReadings = new double[3];
        reactValue = react;
        
        initialize();
    }
    
    public void initialize() {
        // Get initial readings
        
        initialReadings[0] = accel.getAcceleration(ADXL345_I2C.Axes.kX);
        initialReadings[1] = accel.getAcceleration(ADXL345_I2C.Axes.kY);
        initialReadings[2] = accel.getAcceleration(ADXL345_I2C.Axes.kZ);
        //System.out.println(initialReadings[1]);
    }
    
    public double getX() {
        
        return calculateAngle(accel.getAcceleration(ADXL345_I2C.Axes.kX) - initialReadings[0]);
    }
    
    public double getY() {
        //System.out.println(accel.getAcceleration(ADXL345_I2C.Axes.kY));
        //double temp = accel.getAcceleration(ADXL345_I2C.Axes.kY) - initialReadings[1];
        //System.out.println(temp + ":"+initialReadings[1]);
        return calculateAngle(accel.getAcceleration(ADXL345_I2C.Axes.kY) - initialReadings[1]);
    }
    
    public double getZ() {
        return calculateAngle(accel.getAcceleration(ADXL345_I2C.Axes.kZ) - initialReadings[2]);
    }
    
    public boolean isXOffset() {
        return Math.abs(getX()) > reactValue;
    }
    
    public boolean isYOffset() {
        return Math.abs(getY()) > reactValue;
    }
    
    public boolean isZOffset() {
        return Math.abs(getZ()) > reactValue;
    }
    
    /**
     * lol so it returns it in radians
     * @param radianValue
     * @return angle in degrees
     */
    private double calculateAngle(double radianValue){
        return radianValue*57.296;
    }
}
