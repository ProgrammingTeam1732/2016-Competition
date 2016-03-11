package org.usfirst.frc.team1732.statemachine;

public class RobotInstruction {
	public String next = null;
	public boolean machine_finished = false;
		
	public boolean catapult_in = false;
	public boolean catapult_out = false;
	public boolean catapult_latch = false;
	public boolean catapult_release = false;
	
	public boolean fingers_open = false;
	public boolean fingers_close = false;	
	
	public double drive_left = 0;
	public double drive_right = 0;
}