package org.usfirst.frc.team1732.robot;

import edu.wpi.first.wpilibj.Solenoid;

public class Fingers {
	private Solenoid left;
	private Solenoid right;
	
	private final static boolean LEFT_OPEN = true;
	private final static boolean RIGHT_OPEN = true;
	private final static boolean LEFT_CLOSED = false;
	private final static boolean RIGHT_CLOSED = false;
	
	public Fingers() {
		left = new Solenoid(2, 2);
		right = new Solenoid(2, 3);
	}
	
	public void open() {
		left.set(LEFT_OPEN);
		right.set(RIGHT_OPEN);
	}
	
	public void close() {
		left.set(LEFT_CLOSED);
		right.set(RIGHT_CLOSED);
	}
}
