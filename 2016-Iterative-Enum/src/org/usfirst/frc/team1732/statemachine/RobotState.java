package org.usfirst.frc.team1732.statemachine;

public class RobotState {
	public long start_time;
	
	public int drive_right_dist;
	public int drive_left_dist;
	
	public double angle_to_goal;
	public double distance_to_goal;
	public boolean camera_exists;
	
	public double gyro;
	public int manip_encoder;
	
	public boolean shoot;
	public boolean shoot_mode_far;
	public boolean shoot_mode_close;
	public boolean shoot_mode_auto;
	
	public boolean arm_aligned_high;
	public boolean arm_aligned_middle;
	public boolean arm_aligned_low;
	public boolean arm_aligned_auto;
	public boolean arm_aligned_cheval;
	
	public boolean catapult_aligned_out;
	public boolean catapult_aligned_shoot;
	public boolean catapult_aligned_load;
				
	public boolean fingers_closed;
	public boolean fingers_open;
	
	public boolean intake_down;
	public boolean intake_up;

	public boolean reset_catapult;
}
