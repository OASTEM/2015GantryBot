package org.oastem.frc.control;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CANJaguar;

/**
 *
 * @author joyhsu0504
 * @author bnguyen654
 */

public class Pulley(){
    protected static Pulley instance;
    
    protected CANJaguar right, left;
    
    protected int codesPerRev, distPerRev;
    
    protected final int MOVE_CONSTANT = 1/distPerRev;
    
    protected Pulley(){
    }
    
    public static Pulley getInstance(){
        if(instance == null){
            instance = new Pulley();
        }
            return instance;
    }
    public void init(int rightId, int leftId, double p, double i, double d){
        right = new CANJaguar(rightId);
        right.setPositionMode(CANJaguar.kQuadEncoder, codesPerRev, p, i, ,d);
        left = new CANJaguar(leftId);
        left.setPositionMode(CANJaguar.kQuadEncoder, codesPerRev, p, i, d);
    }
    
    public void configEnc(int cpr, int dpr){
        codesPerRev = cpr;
        distPerRev = dpr;
    }
    
    public void goToPos(double pos){
        right.set(pos);
        left.set(pos);
    }

	public boolean upToHook() {
		
	}
	
	public boolean downToUnhook() {
		
	}
	
	public boolean checkHooked() {
		
	}
    
}