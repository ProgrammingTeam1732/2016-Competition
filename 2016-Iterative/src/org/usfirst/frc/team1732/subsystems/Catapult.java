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
	
	private enum Setpoints {
		In(275),		// 250
		Out(1337);		// 2000
		
		private final int _value;
		Setpoints(int value) { _value = value; }
	    public int Value() { return _value; }
	}
	
	private static final int RADIUS = 100;
	private static final boolean LATCHED = false;
	private static final boolean RELEASE = true;
	
	private double previous_error = 0;
	private double integral = 0;
	private Setpoints setpoint = Setpoints.In;
	private double P = 5.0;
	private double I = 0.0;
	private double D = 0.0;
	private double MAX = 0.5;
	
	private Mode mode = Mode.Auto;
	private long time = System.currentTimeMillis();
	
	public Catapult() {
		SmartDashboard.putNumber("Catapult P", P);
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
			
			double dt = (System.currentTimeMillis() - time);
			double measured = pot.getValue();
			SmartDashboard.putNumber("Catapult Pot", measured);
			double error = setpoint.Value() - measured;
			integral += error * dt/1000.0;
			double derivative = (error - previous_error) / dt;
			double output = (P/1000.0) * error + (I/1000.0) * integral + (D/1000.0) * derivative;
			
			motor.set(limit(output));
			
			SmartDashboard.putNumber("Catapult Setpoint", setpoint.Value());
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
	
	public void setIn()		{ setpoint = Setpoints.In;	run(); }	public boolean isIn() 	{ return setpoint == Setpoints.In; }
	public void setOut()	{ setpoint = Setpoints.Out;	run(); }	public boolean isOut()	{ return setpoint == Setpoints.Out; }
	
	public void latch() 	{ latch.set(LATCHED);  SmartDashboard.putBoolean("Latched", true); SmartDashboard.putBoolean("Released", false);}
	public void release() 	{ latch.set(RELEASE); SmartDashboard.putBoolean("Latched", false); SmartDashboard.putBoolean("Released", true); }
	
	public double limit(double in) { if (in > MAX) return MAX; else if (in < -MAX) return -MAX; return in; }
	
	public boolean inDeadbandOut() { return Math.abs(Setpoints.Out.Value() - pot.getValue()) < RADIUS; }
	public boolean inDeadbandIn() { return Math.abs(Setpoints.In.Value() - pot.getValue()) < RADIUS; }
	
	public boolean cocked() {
		return inDeadbandIn() && latch.get() == LATCHED;
	}
	
	public double getPos() {
		return pot.getValue();
	}
}