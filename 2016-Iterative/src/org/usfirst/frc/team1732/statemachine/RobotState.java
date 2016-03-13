package org.usfirst.frc.team1732.statemachine;

public class RobotState {
	public long start_time;
	
	public int drive_right_dist;
	public int drive_left_dist;
	
	public boolean shoot;
	public boolean ball;
	
	public boolean arm_aligned_high;
	public boolean arm_aligned_middle;
	public boolean arm_aligned_low;
	
	public boolean catapult_aligned_out;
	public boolean catapult_aligned_load;
	public boolean catapult_aligned_in;
	
	public boolean low;
	public boolean middle;
	public boolean high;
		
	public boolean next;
	
	public boolean fingers_closed;
	public boolean fingers_open;
}
