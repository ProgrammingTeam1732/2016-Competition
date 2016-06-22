package org.usfirst.frc.team1732.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Fingers {
	private Solenoid solenoid = new Solenoid(2, 0);

	private final static boolean OPEN = true;
	private final static boolean CLOSED = false;
	
	private DigitalInput intake_sensor = new DigitalInput(5);

	public Fingers() {
		SmartDashboard.putBoolean("Fingers Open", solenoid.get() == OPEN);
		SmartDashboard.putBoolean("Fingers Closed", solenoid.get() == CLOSED);
	}

	public void open() {
		solenoid.set(OPEN);
		SmartDashboard.putBoolean("Fingers Open", true);
		SmartDashboard.putBoolean("Fingers Closed", false);
	}

	public void close() {
		solenoid.set(CLOSED);
		SmartDashboard.putBoolean("Fingers Open", false);
		SmartDashboard.putBoolean("Fingers Closed", true);
	}
	
	public boolean isOpen() {
		return solenoid.get() == OPEN;
	}
	
	public boolean isClosed() {
		return solenoid.get() ==  CLOSED;
	}
	
	public boolean hasBall() {
		return intake_sensor.get();
	}
}
