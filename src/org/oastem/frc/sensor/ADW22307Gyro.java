/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.oastem.frc.sensor;

import edu.wpi.first.wpilibj.Gyro;

/**
 * Enclosure class for the Gyro class, customized to deal with the constant
 * drifting of the gyroscope.
 * 
 * @author KTOmega
 */
public class ADW22307Gyro {
    private Gyro g;
    //private static final double DRIFT_PER_SECOND = 0.014578491443051003;
    private static final double DRIFT_PER_SECOND = 0.048;
    private long lastUpdateTime = 0;
    
    public ADW22307Gyro(int port) {
        g = new Gyro(port);
        lastUpdateTime = System.currentTimeMillis();
        g.setSensitivity(7000);
        initialize();
    }
    
    public void initialize() {
        g.reset();
        //g.setSensitivity(0.07);
        lastUpdateTime = System.currentTimeMillis();
    }
    
    /*public double getAngle()
    {
        return g.getAngle();
    }*/
    
    public double getAngle() {
        long currentTime = System.currentTimeMillis();
        
        double value = g.getAngle() - DRIFT_PER_SECOND * (currentTime - lastUpdateTime)/1000.0;
        
        lastUpdateTime = currentTime;
        
        return value;
    }
}
