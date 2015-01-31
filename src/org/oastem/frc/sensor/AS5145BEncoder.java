/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.oastem.frc.sensor;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
/**
 *
 * @author Potato
 */
public class AS5145BEncoder {
    private Encoder e;
    // from the API:
    // 1X Count only the Rising Edge
    // 2X Count both the Rising Edge 
    // 4X Count Rising and Falling on Both channels
    // What does this mean...
    private CounterBase.EncodingType encodingType = CounterBase.EncodingType.k4X;
    
    /**
     * Creates an encoder using the two SIG inputs (A and B) 
     * Also allows an encoder to be in reverse
     * And the encoding type (1X 2X 4X) can be adjusted above
     * @param aPort I/0 SIG A 
     * @param bPort I/0 SIG B
     * @param reverse when true, returned values are inverted
     */
    public AS5145BEncoder(int aPort, int bPort, boolean reverse){
        e = new Encoder(aPort, bPort, reverse, encodingType);
        initialize();
    }
    public void initialize(){
        //e.start(); //ALWAYS START THE ENCODER!!!!111
    	//NOT WITH 2015 RoboRIO!!!!!!!! - mduong15 
        e.reset();
    }
    
    
    // below methods are the get methods associated with the encoder
    // TODO: test what these outputs give us
    // TELL ENGINEERING TO PUT THE MAGNET ON THE ENCODER SO WE CAN DO THIS
    public int getSomething(){
        return e.get();
    }
    public boolean getDirectionThing(){
        return e.getDirection();
    }
    public double getDistanceTravel(){
        return e.getDistance();
    }
    public double getRateThing(){
        return e.getRate();
    }
    public int getRawValue(){
        return e.getRaw();
    }
}
