package org.oastem.frc;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author Mandarker
 */

public class Dashboard {
	// creates a Smartdashboard
	private SmartDashboard board;
	
	// constructor with no parameters that initializes the dashboard
	public Dashboard (){
		board = new SmartDashboard();
	}
	
	/**
	 * @param key: the position of the item on the board, represented by a string key
	 * @param input: the input that the user wants to put in the specific location of the key
	 * The key is not a specific static structure on the SmartDashboard, since it can be moved around.
	 * The key exists in order for the SmartDashboard to search for that specific string identifier
	 * among the objects on the board. 
	 * A Sendable is a base interface of Objects that can be sent across the network.
	 */
	
	//returns a boolean at the position of the key
	public boolean getBoolean(String key){
		return board.getBoolean(key);
	}
	
	// returns a Data object at the position of the key
	public Sendable getData(String key){
		return board.getData(key);
	}
	
	// returns a double at the position of the key
	public double getNumber(String key){
		return board.getNumber(key);
	}
	
	// returns a String at the position of the key 
	public String getString(String key){
		return board.getString(key);
	}
	
	// puts a boolean at the position of key
	public void putBoolean(String key, boolean input){
		board.putBoolean(key, input);
	}
	
	// puts a Data object at the position of key
	public void putData(String key, Sendable input){
		board.putData(key, input);
	}
	
	// puts a double at the position of key
	public void putNumber(String key, double input){
		board.putNumber(key, input);
	}
	
	// puts a String at the position of key
	public void putString(String key, String input){
		board.putString(key, input);
	}
}
