package org.usfirst.frc.team1732.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Catapult {
	private Solenoid latch = new Solenoid(2, 1);
	private CANTalon motor = new CANTalon(17);
	private AnalogInput pot = new AnalogInput(0);
	
	private static final int IN = 		275;	// 250
	private static final int OUT = 		1337;	// 2000
	
	private static final int RADIUS = 100;
	
	private static boolean LATCH = false;
	private static boolean RELEASE = true;
	
	private double previous_error = 0;
	private double integral = 0;
	private double setpoint = IN;
	private long time = System.currentTimeMillis();
	private double P = 5.0;
	private double I = 0.0;
	private double D = 0.0;
	private double MAX = 0.5;
	
	public Catapult() {
		SmartDashboard.putNumber("Catapult P", P);
		SmartDashboard.putNumber("Catapult I", I);
		SmartDashboard.putNumber("Catapult D", D);
		SmartDashboard.putNumber("Catapult MAX", MAX);
		
		SmartDashboard.putBoolean("Latched", false);
		SmartDashboard.putBoolean("Released", false);
		
	}
	
	public void snap() {
		P = SmartDashboard.getNumber("Catapult P", P);
		I = SmartDashboard.getNumber("Catapult I", I);
		D = SmartDashboard.getNumber("Catapult D", D);
		MAX = SmartDashboard.getNumber("Catapult MAX", MAX);
		
		double dt = (System.currentTimeMillis() - time);
		double measured = pot.getValue();
		SmartDashboard.putNumber("Catapult Pot", measured);
		double error = setpoint - measured;
		integral += error * dt/1000.0;
		double derivative = (error - previous_error) / dt;
		double output = (P/1000.0) * error + (I/1000.0) * integral + (D/1000.0) * derivative;
		
		motor.set(limit(output));
		
		SmartDashboard.putNumber("Catapult Setpoint", setpoint);
		SmartDashboard.putNumber("Catapult Output", limit(output));
		SmartDashboard.putNumber("Catapult Error (P)", error);
		SmartDashboard.putNumber("Catapult Integral (I)", integral);
		SmartDashboard.putNumber("Catapult Derivative (D)", derivative);
		
		previous_error = error;
		time = System.currentTimeMillis();
	}
	
	//public void calibrate(double input) { motor.set(input); SmartDashboard.putNumber("Catapult Output", input); SmartDashboard.putNumber("Catapult Pot", pot.getValue()); }
	
	public void setIn()		{ setpoint = IN;	snap(); }	public boolean isIn() 	{ return setpoint == IN; }
	public void setOut()	{ setpoint = OUT;	snap(); }	public boolean isOut()	{ return setpoint == OUT; }
	
	public void latch() 	{ latch.set(LATCH);  SmartDashboard.putBoolean("Latched", true); SmartDashboard.putBoolean("Released", false);}
	public void release() 	{ latch.set(RELEASE); SmartDashboard.putBoolean("Latched", false); SmartDashboard.putBoolean("Released", true); }
	
	public double limit(double in) {
		if (in > MAX) return MAX;
		else if (in < -MAX) return -MAX;
		return in;
	}
	
	public boolean inDeadbandOut() {
		return Math.abs(OUT - pot.getValue()) < RADIUS;
	}
	public boolean inDeadbandIn() {
		return Math.abs(IN - pot.getValue()) < RADIUS;
	}
	
	public double getPos() {
		return pot.getValue();
	}
}
