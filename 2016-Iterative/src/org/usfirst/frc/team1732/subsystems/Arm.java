package org.usfirst.frc.team1732.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm {
	private CANTalon motor = new CANTalon(18);
	private AnalogInput pot = new AnalogInput(1);

	private double previous_error = 0;
	private double previous = 0;
	private double integral = 0;
	private long time = System.currentTimeMillis();

	private double P = -8.0;
	private double I = 0.0;
	private double D = 0.0;
	private double MAX = 0.8;
	private double MIN = 0.2;
	private double RAMP = 0.5; 

	private Mode mode = Mode.AutoHigh;
	private int setpoint = High;

	private static final int RADIUS = 50;
	private static final double UP_SPEED = 0.4;
	private static final double DOWN_SPEED = -0.4;
	private static final double STOP = 0;

	private enum Mode {
		AutoHigh, AutoMiddle, AutoLow, ManualUp, ManualDown, ManualStop
	}

	static int Low = (750); // 500
	static int Middle = (2500); // 1500
	static int High = (3150); // 2800

	public Arm() {
		SmartDashboard.putNumber("Arm P", P);
		SmartDashboard.putNumber("Arm Low", Low);
		SmartDashboard.putNumber("Arm Middle", Middle);
		SmartDashboard.putNumber("Arm High", High);
		SmartDashboard.putNumber("Arm I", I);
		SmartDashboard.putNumber("Arm D", D);
		SmartDashboard.putNumber("Arm MAX", MAX);
		SmartDashboard.putNumber("Arm RAMP", RAMP);
		
	}

	public void run() {
		if (mode == Mode.AutoHigh || mode == Mode.AutoMiddle || mode == Mode.AutoLow) {
			P = SmartDashboard.getNumber("Arm P", P);
			I = SmartDashboard.getNumber("Arm I", I);
			D = SmartDashboard.getNumber("Arm D", D);
			MAX = SmartDashboard.getNumber("Arm MAX", MAX);
			RAMP = SmartDashboard.getNumber("Arm RAMP", RAMP);
			
			Low = (int) SmartDashboard.getNumber("Arm Low", Low);
			Middle = (int) SmartDashboard.getNumber("Arm Middle", Middle);
			High = (int) SmartDashboard.getNumber("Arm High", High);

			double measured = pot.getValue();
			SmartDashboard.putNumber("Arm Pot", measured);
			double dt = (System.currentTimeMillis() - time);
			double error = setpoint - measured;
			integral += error * dt / 1000.0;
			double derivative = (error - previous_error) / dt;
			double output = (P / 1000.0) * error + (I / 1000.0) * integral + (D / 1000.0) * derivative;
			
			if (output < 1) {
				if (previous - output > RAMP/1000 * dt) {
					output = previous - (RAMP/1000 * dt);
				}
			} else {
				if (output - previous > RAMP/1000 * dt) {
					output =  previous + (RAMP/1000 * dt);
				}
			}
			
			if (isHigh() && inDeadbandHigh()) output = 0;
			else if (isMiddle() && inDeadbandMiddle()) output = 0;
			else if (isLow() && inDeadbandLow()) output = 0;
			
			motor.set(limit(limit_low(output)));

			SmartDashboard.putNumber("Arm Setpoint", setpoint);
			SmartDashboard.putNumber("Arm Output", limit(limit_low(output)));
			SmartDashboard.putNumber("Arm Error (P)", error);
			SmartDashboard.putNumber("Arm Integral (I)", integral);
			SmartDashboard.putNumber("Arm Derivative (D)", derivative);

			previous = output;
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

	public void setUp() {
		mode = Mode.ManualUp;
	}

	public void setDown() {
		mode = Mode.ManualDown;
	}

	public void stop() {
		mode = Mode.ManualStop;
	}

	public void setLow() {
		setpoint = Low;
		run();
	}

	public boolean isLow() {
		return setpoint == Low;
	}

	public void setMiddle() {
		setpoint = Middle;
		run();
	}

	public boolean isMiddle() {
		return setpoint == Middle;
	}

	public void setHigh() {
		setpoint = High;
		run();
	}

	public boolean isHigh() {
		return setpoint == High;
	}

	public double limit_low(double in) {
		if (in < MIN && in > -1 * MIN) return 0;
		else return in;
	}
	
	public double limit(double in) {
		if (in > MAX)
			return MAX;
		else if (in < -MAX)
			return -MAX;
		return in;
	}

	public boolean inDeadbandHigh() {
		return Math.abs(High - pot.getValue()) < RADIUS;
	}

	public boolean inDeadbandMiddle() {
		return Math.abs(Middle - pot.getValue()) < 2*RADIUS;
	}

	public boolean inDeadbandLow() {
		return Math.abs(Low - pot.getValue()) < RADIUS;
	}

	public double getPos() {
		return pot.getValue();
	}
}