package org.usfirst.frc.team1732.statemachine;

public class RobotInstruction {
	public String next = null;
	public boolean machine_finished = false;
		
	public boolean catapult_load = false;
	public boolean catapult_in = false;
	public boolean catapult_out = false;
	public boolean catapult_latch = false;
	public boolean catapult_release = false;
	
	public boolean fingers_open = false;
	public boolean fingers_close = false;	
	
	public double drive_left = 0;
	public double drive_right = 0;
	
	public boolean intake_down = false;
	public boolean intake_up = false;
	public boolean intake_in = false;
	public boolean intake_out = false;
	
	public boolean arm_high = false;
	public boolean arm_middle = false;
	public boolean arm_low = false;
	
	public boolean defense_down = false;
	public boolean defense_up = false;
	
}