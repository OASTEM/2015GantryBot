package org.oastem.frc.control;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author joyhsu0504
 * @author bnguyen654
 */


public class Pulley{
    private static Pulley instance;
    
    private CANJaguar right, left;
    private final byte SYNC_GROUP = 0x0008;
    
    private int codesPerRev = 497,
    		distPerRev = 0; //to be checked
    
    //values are to be inches
    private final double AUTO_MOVE_DIST = 1;
    
    private final double FORWARD_LIMIT = 1,
    		REVERSE_LIMIT = 1;
    
    private Pulley(){
    	
    }
    
    public static Pulley getInstance(){
        if(instance == null){
            instance = new Pulley();
        }
            return instance;
    }
    
    public void init(int rightId, int leftId, double p, double i, double d){
       init(new CANJaguar(rightId), new CANJaguar(leftId), p, i, d);
    }
    public void init(CANJaguar r, CANJaguar l, double p, double i, double d){
        right = r;
        right.setPositionMode(CANJaguar.kQuadEncoder, codesPerRev, p, i, d);
        right.configSoftPositionLimits(FORWARD_LIMIT, REVERSE_LIMIT);
        right.enableControl();
        
        left = l;
        left.setPositionMode(CANJaguar.kQuadEncoder, codesPerRev, p, i, d);
        left.configSoftPositionLimits(FORWARD_LIMIT, REVERSE_LIMIT);
        left.enableControl();
    }
    
    public void configEnc(int cpr, int dpr){
        codesPerRev = cpr;
        distPerRev = dpr;
    }
    
    public void up(double dist){
    	right.set(right.getPosition() + i2r(dist),SYNC_GROUP);
    	left.set(right.getPosition() + i2r(dist),SYNC_GROUP);
    	CANJaguar.updateSyncGroup(SYNC_GROUP);
    }
    public void down(double dist){
    	right.set(right.getPosition() - i2r(dist),SYNC_GROUP);
    	left.set(right.getPosition() - i2r(dist),SYNC_GROUP);
    	CANJaguar.updateSyncGroup(SYNC_GROUP);
    }
   /** public void doSlave(){
    	left.set(right.getPosition() + i2r(0.5));
    }*/
    
    public void up(){
    	up(AUTO_MOVE_DIST);
    }
    
    public void down(){
    	down(AUTO_MOVE_DIST);
    }
    
    public void goToPos(double pos){
        right.set(i2r(pos),SYNC_GROUP);
        left.set(i2r(pos),SYNC_GROUP);
        CANJaguar.updateSyncGroup(SYNC_GROUP);
    }
    
    public void debug(SmartDashboard dash){
    	dash.putNumber("Left Enc", left.getPosition());
    	dash.putNumber("Right Enc", right.getPosition());
    	dash.putNumber("Left Setpoint", left.get());
    	dash.putNumber("Left Setpoint", left.get());
    }

	public boolean upToHook() {
		return false;
	}
	
	public boolean downToUnhook() {
		return false;
	}
	
	public boolean checkHooked() {
		return false;
	}
	private double r2i(double rev){ //rev to in
		return rev * distPerRev;
	}
	private double i2r(double in){ //in to rev
		return in/distPerRev;
	}
}
