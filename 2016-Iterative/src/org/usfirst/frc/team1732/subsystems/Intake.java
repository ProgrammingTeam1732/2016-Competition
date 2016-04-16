package org.usfirst.frc.team1732.subsystems;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
	private CANTalon motor = new CANTalon(12);
	private Solenoid solenoid = new Solenoid(2, 3);

	private static final boolean UP = false;
	private static final boolean DOWN = true;

	private static final double INTAKE_SPEED = 0.60;
	private static final double OUTPUT_SPEED = -0.80;
	private static final double STOP = 0.0;
	
	public void setIn() 	{ motor.set(INTAKE_SPEED); 	SmartDashboard.putBoolean("Intake In", true); 	SmartDashboard.putBoolean("Intake Out", false); }
	public void setOut() 	{ motor.set(OUTPUT_SPEED); 	SmartDashboard.putBoolean("Intake In", false); 	SmartDashboard.putBoolean("Intake Out", true); }
	public void setStop() 	{ motor.set(STOP); 			SmartDashboard.putBoolean("Intake In", false); 	SmartDashboard.putBoolean("Intake Out", false); }
	
	public void setUp() 	{ solenoid.set(UP); 		SmartDashboard.putBoolean("Intake Up", true); 	SmartDashboard.putBoolean("Intake Down", false); }
	public void setDown() 	{ solenoid.set(DOWN); 		SmartDashboard.putBoolean("Intake Up", false); 	SmartDashboard.putBoolean("Intake Down", true); }
	
	public boolean isUp()	{ return solenoid.get() == UP; }
	public boolean isDown()	{ return solenoid.get() == DOWN; }
}