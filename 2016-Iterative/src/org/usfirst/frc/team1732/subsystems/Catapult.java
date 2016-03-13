package org.usfirst.frc.team1732.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Catapult {
	private Solenoid latch = new Solenoid(2, 1);
	private CANTalon motor = new CANTalon(17);
	private AnalogInput pot = new AnalogInput(0);
	
	private enum Mode { Auto, Manual }
	
	static int Load = 350; 
	static int In = (620);		// 250
	static int Out = (1800);		// 2000
		
	
	private static final int RADIUS = 100;
	private static final boolean LATCHED = false;
	private static final boolean RELEASE = true;
	
	private double previous_error = 0;
	private double integral = 0;
	private int setpoint = In;
	private double P = 5.0;
	private double I = 0.0;
	private double D = 0.0;
	private double MAX = 0.6;
	
	private Mode mode = Mode.Auto;
	private long time = System.currentTimeMillis();
	
	public Catapult() {
		SmartDashboard.putNumber("Catapult P", P);
		SmartDashboard.putNumber("Catapult In", In);
		SmartDashboard.putNumber("Catapult Out", Out);
		SmartDashboard.putNumber("Catapult I", I);
		SmartDashboard.putNumber("Catapult D", D);
		SmartDashboard.putNumber("Catapult MAX", MAX);
		
		SmartDashboard.putBoolean("Latched", false);
		SmartDashboard.putBoolean("Released", false);
		
	}
	
	public void run() {
		if (mode == Mode.Auto) {
			P = SmartDashboard.getNumber("Catapult P", P);
			I = SmartDashboard.getNumber("Catapult I", I);
			D = SmartDashboard.getNumber("Catapult D", D);
			MAX = SmartDashboard.getNumber("Catapult MAX", MAX);
			In = (int) SmartDashboard.getNumber("Catapult In", In);
			Out = (int) SmartDashboard.getNumber("Catapult Out", Out);
			
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
		} else { 
			motor.set(0);
		}
	}
	
	//public void calibrate(double input) { motor.set(input); SmartDashboard.putNumber("Catapult Output", input); SmartDashboard.putNumber("Catapult Pot", pot.getValue()); }
	
	public void setLoad()	{ setpoint = Load;	run(); }	public boolean isLoad()	{ return setpoint == Load; }
	public void setIn()		{ setpoint = In;	run(); }	public boolean isIn() 	{ return setpoint == In; }
	public void setOut()	{ setpoint = Out;	run(); }	public boolean isOut()	{ return setpoint == Out; }
	
	public void latch() 	{ latch.set(LATCHED);  SmartDashboard.putBoolean("Latched", true); SmartDashboard.putBoolean("Released", false);}
	public void release() 	{ latch.set(RELEASE); SmartDashboard.putBoolean("Latched", false); SmartDashboard.putBoolean("Released", true); }
	
	public double limit(double in) { if (in > MAX) return MAX; else if (in < -MAX) return -MAX; return in; }
	
	public boolean inDeadbandOut() { return Math.abs(Out - pot.getValue()) < RADIUS; }
	public boolean inDeadbandIn() { return Math.abs(In - pot.getValue()) < RADIUS; }
	public boolean inDeadbandLoad() { return Math.abs(Load - pot.getValue()) < RADIUS; }
	
	public boolean cocked() {
		return inDeadbandIn() && latch.get() == LATCHED;
	}
	
	public double getPos() {
		return pot.getValue();
	}
}
