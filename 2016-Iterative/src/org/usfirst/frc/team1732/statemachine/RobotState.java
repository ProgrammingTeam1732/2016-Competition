package org.usfirst.frc.team1732.statemachine;

public class RobotState {
	public long start_time;
	
	public int drive_right_dist;
	public int drive_left_dist;
	
	//public double camera_angle;
	//public double distance_to_goal;
	
	public double gyro;
	
	public int manip_encoder;
		
	public boolean shoot;
		
	public boolean arm_aligned_high;
	public boolean arm_aligned_middle;
	public boolean arm_aligned_low;
	public boolean arm_aligned_auto;
	
	public boolean catapult_aligned_out;
	public boolean catapult_aligned_load;
	public boolean catapult_aligned_in;
				
	public boolean fingers_closed;
	public boolean fingers_open;
	
	public boolean intake_down;
	public boolean intake_up;
}
