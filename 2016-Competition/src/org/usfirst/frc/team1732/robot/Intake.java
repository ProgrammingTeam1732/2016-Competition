package org.usfirst.frc.team1732.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake {
	private CANTalon motor;
	private Solenoid left;
	private Solenoid right;
	
	private static final boolean LEFT_UP = true;
	private static final boolean RIGHT_UP = true;
	private static final boolean LEFT_DOWN = false;
	private static final boolean RIGHT_DOWN = false;
	
	private static final double INTAKE_SPEED = -0.5;
	private static final double OUTPUT_SPEED = 0.5;
	private static final double STOP = 0.0;
	
	public Intake() {
		motor = new CANTalon(10);
		left = new Solenoid(2, 0);
		right = new Solenoid(2, 1);
	}
	
	public void setIn() {
		motor.set(INTAKE_SPEED);
	}
	
	public void setOut() {
		motor.set(OUTPUT_SPEED);
	}
	
	public void setStop() {
		motor.set(STOP);
	}
	
	public void setUp() {
		left.set(LEFT_UP);
		right.set(RIGHT_UP);
	}
	
	public void setDown() {
		left.set(LEFT_DOWN);
		right.set(RIGHT_DOWN);
	}
	
	public void disable() {
		motor.set(STOP);
	}
}
