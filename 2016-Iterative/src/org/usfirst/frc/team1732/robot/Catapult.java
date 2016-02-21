package org.usfirst.frc.team1732.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Catapult {
	private Solenoid latch = new Solenoid(2, 1);
	private CANTalon motor = new CANTalon(17);
	private AnalogInput pot = new AnalogInput(1);
	
	private static final int IN = 		500;
	private static final int OUT = 		2000;
	
	private static final int RADIUS = 100;
	
	private static boolean LATCH = true;
	private static boolean RELEASE = false;
	
	private double previous_error = 0;
	private double integral = 0;
	private double setpoint = IN;
	private long time = System.currentTimeMillis();
	private double P = 1.0;
	private double I = 1.0;
	private double D = 1.0;
	
	public Catapult() {
		SmartDashboard.putNumber("Catapult P", P);
		SmartDashboard.putNumber("Catapult I", I);
		SmartDashboard.putNumber("Catapult D", D);
	}
	
	public void snap() {
		P = SmartDashboard.getNumber("Catapult P", P);
		I = SmartDashboard.getNumber("Catapult I", I);
		D = SmartDashboard.getNumber("Catapult D", D);
		
		double dt = (System.currentTimeMillis() - time);
		double measured = pot.getValue();
		SmartDashboard.putNumber("Catapult Pot", measured);
		double error = setpoint - measured;
		integral += error * dt;
		double derivative = (error - previous_error) / dt;
		double output = P * error + I * integral + D * derivative;
		
		motor.set(output);
		
		SmartDashboard.putNumber("Catapult Setpoint", setpoint);
		SmartDashboard.putNumber("Catapult PID Output", output);
		SmartDashboard.putNumber("Catapult Error (P)", error);
		SmartDashboard.putNumber("Catapult Integral (I)", integral);
		SmartDashboard.putNumber("Catapult Derivative (D)", derivative);
		
		previous_error = error;
		time = System.currentTimeMillis();
	}
	
	public void calibrate(double input) { motor.set(input); SmartDashboard.putNumber("Catapult Output", input); SmartDashboard.putNumber("Catapult Pot", pot.getValue()); }
	
	public void setIn()	{ setpoint = IN;	snap(); }		public boolean isIn() 	{ return setpoint == IN; }
	public void setOut()	{ setpoint = OUT;	snap(); }	public boolean isOut()	{ return setpoint == OUT; }
	
	public void latch() 	{ latch.set(LATCH); }
	public void release() 	{ latch.set(RELEASE); }
	
	public boolean inDeadband() {
		return Math.abs(setpoint - pot.getValue()) < RADIUS;
	}

}
