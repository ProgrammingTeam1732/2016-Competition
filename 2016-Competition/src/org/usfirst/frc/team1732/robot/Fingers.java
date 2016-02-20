package org.usfirst.frc.team1732.robot;

import edu.wpi.first.wpilibj.Solenoid;

public class Fingers {
	private Solenoid solenoid;
	
	private final static boolean OPEN = true;
	private final static boolean CLOSED = false;
	
	public Fingers() {
		solenoid = new Solenoid(2, 0);
	}
	
	public void open() {
		solenoid.set(OPEN);
	}
	
	public void close() {
		solenoid.set(CLOSED);
	}
}
