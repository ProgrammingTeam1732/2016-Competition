package org.usfirst.frc.team1732.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm {
	private CANTalon motor = new CANTalon(18);
	private AnalogInput pot = new AnalogInput(1);	
	
	private double previous_error = 0;
	private double integral = 0;
	private long time = System.currentTimeMillis();
	
	private double P = 1.0;
	private double I = 0.0;
	private double D = 0.0;
	private double MAX = 0.5;
	
	private Mode mode = Mode.AutoHigh;
	private Setpoints setpoint = Setpoints.High;
	
	private static final int RADIUS = 100;
	private static final double UP_SPEED = 0.4;
	private static final double DOWN_SPEED = -0.4;
	private static final double STOP = 0;
	
	private enum Mode {
		AutoHigh,
		AutoMiddle,
		AutoLow,
		ManualUp,
		ManualDown,
		ManualStop
	}
	
	private enum Setpoints {
		Low(250),		// 500
		Middle(1300),	// 1500
		High(1800);		// 2800
		
		private final int _value;
		Setpoints(int value) { _value = value; }
	    public int Value() { return _value; }
	}
	
	public Arm() { SmartDashboard.putNumber("Arm P", P); SmartDashboard.putNumber("Arm I", I); SmartDashboard.putNumber("Arm D", D); SmartDashboard.putNumber("Arm MAX", MAX); }
	
	public void run() {
		if (mode == Mode.AutoHigh || mode == Mode.AutoMiddle || mode == Mode.AutoLow) {
			P = SmartDashboard.getNumber("Arm P", P);
			I = SmartDashboard.getNumber("Arm I", I);
			D = SmartDashboard.getNumber("Arm D", D);
			MAX = SmartDashboard.getNumber("Arm MAX", MAX);
			
			double measured = pot.getValue();
			SmartDashboard.putNumber("Arm Pot", measured);
			double dt = (System.currentTimeMillis() - time);
			double error = setpoint.Value() - measured;
			integral += error * dt/1000.0;
			double derivative = (error - previous_error) / dt;
			double output = (P/1000.0) * error + (I/1000.0) * integral + (D/1000.0) * derivative;
			
			motor.set(limit(output));
			
			SmartDashboard.putNumber("Arm Setpoint", setpoint.Value());
			SmartDashboard.putNumber("Arm Output", output);
			SmartDashboard.putNumber("Arm Error (P)", error);
			SmartDashboard.putNumber("Arm Integral (I)", integral);
			SmartDashboard.putNumber("Arm Derivative (D)", derivative);
			
			previous_error = error;
			time = System.currentTimeMillis();
		} else {
			if (mode == Mode.ManualUp) {
				motor.set(UP_SPEED);
			} else if (mode == Mode.ManualDown) {
				motor.set(DOWN_SPEED);
			} else {
				motor.set(STOP);
			}
		}
	}
	
	public void setUp()		{ mode = Mode.ManualUp; }
	public void setDown()	{ mode = Mode.ManualDown; }
	public void stop()		{ mode = Mode.ManualStop; }
	
	public void setLow()	{ setpoint = Setpoints.Low;		run(); }	public boolean isLow() 		{ return setpoint == Setpoints.Low; }
	public void setMiddle()	{ setpoint = Setpoints.Middle; 	run(); }	public boolean isMiddle() 	{ return setpoint == Setpoints.Middle; }
	public void setHigh()	{ setpoint = Setpoints.High;	run(); }	public boolean isHigh() 	{ return setpoint == Setpoints.High; }
	
	public double limit(double in) { if (in > MAX) return MAX; else if (in < -MAX) return -MAX; return in; }
	public boolean inDeadbandHigh() { return Math.abs(Setpoints.High.Value() - pot.getValue()) < RADIUS; }
	public boolean inDeadbandMiddle() { return Math.abs(Setpoints.Middle.Value() - pot.getValue()) < RADIUS; }
	public boolean inDeadbandLow() { return Math.abs(Setpoints.Low.Value() - pot.getValue()) < RADIUS; }
	public double getPos() { return pot.getValue(); }
}