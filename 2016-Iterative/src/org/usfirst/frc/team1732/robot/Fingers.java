package org.usfirst.frc.team1732.robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Fingers {
	private Solenoid solenoid = new Solenoid(2, 0);
	
	private final static boolean OPEN = true;
	private final static boolean CLOSED = false;
		
	public void open()	{ solenoid.set(OPEN); 	SmartDashboard.putBoolean("Fingers Open", true); SmartDashboard.putBoolean("Fingers Closed", false);}
	public void close()	{ solenoid.set(CLOSED);	SmartDashboard.putBoolean("Fingers Open", false); SmartDashboard.putBoolean("Fingers Closed", true);}
}
