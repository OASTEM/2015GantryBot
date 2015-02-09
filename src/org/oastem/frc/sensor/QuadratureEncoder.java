/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.oastem.frc.sensor;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;

/**
 *
 * @author mduong15
 */
public class QuadratureEncoder {
    private final Encoder enc;
    
    //This is different depending on encoder and motors (account for gear ratio).
    //This must be figured out by using docs and specs.
    //(If desperate, just have the motor spin one rotation and see how many pulses it is.)
    private double pulsesPerRevolution;
    
    
    /**
     * Creates an encoder using the two SIG inputs (A and B).
     * Default to use to just get encoder readings and not use getDistance().
     * @param channelA I/0 SIG A.
     * @param channelB I/0 SIG B.
     */
    public QuadratureEncoder(int channelA, int channelB)
    {
        enc = new Encoder(channelA, channelB);
    }
    
    /**
     * Creates an encoder using the two SIG inputs (A and B) and index.
     * Default to use to just get encoder readings and not use getDistance().
     * @param channelA I/0 SIG A.
     * @param channelB I/0 SIG B.
     * @param index I/0 SIG Index.
     */
    public QuadratureEncoder(int channelA, int channelB, int index)
    {
        enc = new Encoder(channelA, channelB, index);
    }
    
    /**
     * Creates an encoder using the two SIG inputs (A and B) .
     * pulsesPerRev is used for getDistance() methods.
     * @param channelA I/0 SIG A.
     * @param channelB I/O SIG B.
     * @param pulsesPerRev Number of pulses for a revolution of the motor (look at instance variable).
     */
    public QuadratureEncoder(int channelA, int channelB, double pulsesPerRev)
    {
        enc = new Encoder(channelA, channelB);
        pulsesPerRevolution = pulsesPerRev;
    }
    
    /**
     * Creates an encoder using the two SIG inputs (A and B).
     * Also allows the encoder to be reversed.
     * pulsesPerRev is used for getDistance() methods.
     * @param channelA I/0 SIG A.
     * @param channelB I/O SIG B.
     * @param isReversed When true, returned values are inverted.
     * @param pulsesPerRev Number of pulses for a revolution of the motor (look at instance variable).
     */
    public QuadratureEncoder(int channelA, int channelB, boolean isReversed, double pulsesPerRev)
    {
        enc = new Encoder(channelA, channelB, isReversed);
        pulsesPerRevolution = pulsesPerRev;
    }
    
    /**
     * Creates an encoder using the two SIG inputs (A and B).
     * Also allows the encoder to be reversed.
     * Can also control the difference between getRaw and get values.
     * pulsesPerRev is used for getDistance() methods.
     * @param channelA I/0 SIG A.
     * @param channelB I/O SIG B.
     * @param isReversed When true, returned values are inverted.
     * @param scaleValue getRaw() values are divided by multiples of 1, 2, or 4 to increase accuracy.
     * @param pulsesPerRev Number of pulses for a revolution of the motor (look at instance variable).
     */
    public QuadratureEncoder(int channelA, int channelB, boolean isReversed,
                            int scaleValue, double pulsesPerRev)
    {
        CounterBase.EncodingType encType = CounterBase.EncodingType.k4X;
        
        if (scaleValue == 1)
            encType = CounterBase.EncodingType.k1X;
        else if (scaleValue == 2)
            encType = CounterBase.EncodingType.k2X;
        else if (scaleValue == 4)
            encType = CounterBase.EncodingType.k4X;
        
        enc = new Encoder(channelA, channelB, isReversed, encType);
        
        pulsesPerRevolution = pulsesPerRev;
    }
    
    /**
     * Creates an encoder using the two SIG inputs (A and B).
     * Can also control the difference between getRaw and get values.
     * pulsesPerRev is used for getDistance() methods.
     * @param channelA I/0 SIG A.
     * @param channelB I/O SIG B.
     * @param scaleValue getRaw() values are divided by multiples of 1, 2, or 4 to increase accuracy.
     * @param pulsesPerRev Number of pulses for a revolution of the motor (look at instance variable).
     */
    public QuadratureEncoder(int channelA, int channelB, int scaleValue, double pulsesPerRev)
    {
        this(channelA, channelB, false, scaleValue, pulsesPerRev);
    }
    
    /**
     * Free the resources used by this object.
     */
    public void free()
    {
        enc.free();
    }
    
    /**
     * Gets the current count, accounting for the scale.
     * @return Current count from the Encoder adjusted for the 1x, 2x, or 4x scale factor.
     */
    public int get()
    {
        return enc.get();
    }
    
    /**
     * Gets the distance the robot has driven since the last reset,
     * scaled by the value from setDistancePerPulse().
     * @return The distance driven since the last reset.
     */
    public double getDistance()
    {
        return enc.getDistance();
    }
    
    /**
     * Get the current rate of the encoder.
     * Units are distance per second.
     * The distance unit is whatever unit was used for setDistancePerPulse().
     * @return The rate in which the encoder is spinning.
     */
    public double getRate()
    {
        return enc.getRate();
    }
    
    /**
     * Gets the raw value from the encoder.
     * This does not account for the scale.
     * @return The current raw count from the encoder.
     */
    public int getRaw()
    {
        return enc.getRaw();
    }
    
    /**
     * Returns if the encoder is going forward or not, accounting for reflections.
     * If the encoder stops, it will continue to return the last direction it was spinning.
     * @return True if the motor is going forward, false if backward.
     */
    public boolean isGoingForward()
    {
        return enc.getDirection();
    }
    
    /**
     * Determine if the encoder is stopped.
     * We may need to add setMaxPeriod() or setMinPeriod() methods to wrapper if method does not work.
     * @return True if the encoder is considered stopped.
     */
    public boolean isStopped()
    {
        return enc.getStopped();
    }
    
    /**
     * Resets the encoder values to zero.
     * This means distance will also be zero.
     */
    public void reset()
    {
        enc.reset();
    }
    
    /**
     * Sets the encoder to read each pulse as a certain amount of distance.
     * Used to set up for getDistance(), so set it with constructor.
     * To find distancePerDriverRevolution:
     * Measure the distance gone in one rotation of the shaft connected to the encoder.
     * This distance can be any unit, but this will be the unit used in getDistance().
     * @param distancePerDriverRevolution Distance traveled in one rotation of the encoder shaft.
     */
    public void setDistancePerPulse(int distancePerDriverRevolution)
    {
        enc.setDistancePerPulse(distancePerDriverRevolution / pulsesPerRevolution);
    }
    
}
