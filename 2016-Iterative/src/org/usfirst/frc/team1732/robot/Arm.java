package org.usfirst.frc.team1732.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm {
	private CANTalon motor = new CANTalon(18);
	private AnalogInput pot = new AnalogInput(1);
		
	private static final int LOW = 		500;
	private static final int MIDDLE = 	1500;
	private static final int HIGH = 	2500;
	
	private static final int RADIUS = 100;
	
	private double previous_error = 0;
	private double integral = 0;
	private double setpoint = HIGH;
	private long time = System.currentTimeMillis();
	private double P = 1.0;
	private double I = 0.0;
	private double D = 0.0;
	
	public Arm() {
		SmartDashboard.putNumber("Arm P", P);
		SmartDashboard.putNumber("Arm I", I);
		SmartDashboard.putNumber("Arm D", D);
	}
	
	public void snap() {
		P = SmartDashboard.getNumber("Arm P", P);
		I = SmartDashboard.getNumber("Arm I", I);
		D = SmartDashboard.getNumber("Arm D", D);
		
		double measured = pot.getValue();
		SmartDashboard.putNumber("Arm Pot", measured);
		double dt = (System.currentTimeMillis() - time);
		double error = setpoint - measured;
		integral += error * dt/1000.0;
		double derivative = (error - previous_error) / dt;
		double output = (P/1000.0) * error + (I/1000.0) * integral + (D/1000.0) * derivative;
		
		motor.set(output);
		
		SmartDashboard.putNumber("Arm Setpoint", setpoint);
		SmartDashboard.putNumber("Arm Output", output);
		SmartDashboard.putNumber("Arm Error (P)", error);
		SmartDashboard.putNumber("Arm Integral (I)", integral);
		SmartDashboard.putNumber("Arm Derivative (D)", derivative);
		
		previous_error = error;
		time = System.currentTimeMillis();
	}
	
	public void calibrate(double input) { motor.set(input); SmartDashboard.putNumber("Arm Output", input); SmartDashboard.putNumber("Arm Pot", pot.getValue()); }
	
	public void setLow()	{ setpoint = LOW;		snap(); }	public boolean isLow() 		{ return setpoint == LOW; }
	public void setMiddle()	{ setpoint = MIDDLE;	snap(); }	public boolean isMiddle() 	{ return setpoint == MIDDLE; }
	public void setHigh()	{ setpoint = HIGH;		snap(); }	public boolean isHigh() 	{ return setpoint == HIGH; }
	
	public boolean inDeadband() {
		return Math.abs(setpoint - pot.getValue()) < RADIUS;
	}
}
