package org.usfirst.frc.team1732.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake {
	private CANTalon motor;
	private Solenoid solenoid;
	
	private static final boolean UP = true;
	private static final boolean DOWN = false;
	
	private static final double INTAKE_SPEED = -0.5;
	private static final double OUTPUT_SPEED = 0.5;
	private static final double STOP = 0.0;
	
	public Intake() {
		motor = new CANTalon(10);
		solenoid = new Solenoid(2, 3);
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
		solenoid.set(UP);
	}
	
	public void setDown() {
		solenoid.set(DOWN);
	}
	
	public void disable() {
		motor.set(STOP);
	}
}
