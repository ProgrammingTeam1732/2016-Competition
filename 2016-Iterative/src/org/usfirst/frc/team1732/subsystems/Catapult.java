package org.usfirst.frc.team1732.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Catapult {
	private Solenoid latch = new Solenoid(2, 1);
	private CANTalon motor = new CANTalon(18);
	private AnalogInput pot = new AnalogInput(0);

	private static int Load = 250;
	private static int Auto = 270;
	private static int Far = 700; // 500 practice, 460 competition old, 720
	private static int Close = 900; // 650 practice, 1150 competition old, 1150 competition new
	private static int Out = 2020; // 2000 is competition, 1750 is practice
	private static int Shoot = 500;
	// before out was 1930
	// tset

	private static final int RADIUS = 100;
	private static final boolean LATCHED = false;
	private static final boolean RELEASE = true;

	private double previous_error = 0;
	private double integral = 0;
	private int setpoint = Load;
	private double P = 6.0;
	private double I = 3.0;
	private double D = 0.0;
	private double MAX = 0.6;

	private long time = System.currentTimeMillis();

	public Catapult() {
		SmartDashboard.putNumber("Catapult P", P);
		SmartDashboard.putNumber("Catapult Far", Far);
		SmartDashboard.putNumber("Catapult Close", Close);
		SmartDashboard.putNumber("Catapult Auto", Auto);
		SmartDashboard.putNumber("Catapult Out", Out);
		SmartDashboard.putNumber("Catapult Load", Load);
		SmartDashboard.putNumber("Catapult I", I);
		SmartDashboard.putNumber("Catapult D", D);
		SmartDashboard.putNumber("Catapult MAX", MAX);

		SmartDashboard.putBoolean("Latched", false);
		SmartDashboard.putBoolean("Released", false);

	}

	public void run() {
		P = SmartDashboard.getNumber("Catapult P", P);
		I = SmartDashboard.getNumber("Catapult I", I);
		D = SmartDashboard.getNumber("Catapult D", D);
		MAX = SmartDashboard.getNumber("Catapult MAX", MAX);
		Far = (int) SmartDashboard.getNumber("Catapult Far", Far);
		Close = (int) SmartDashboard.getNumber("Catapult Close", Close);
		Out = (int) SmartDashboard.getNumber("Catapult Out", Out);
		Load = (int) SmartDashboard.getNumber("Catapult Load", Load);
		double dt = (System.currentTimeMillis() - time);
		double measured = pot.getValue();
		SmartDashboard.putNumber("Catapult Pot", measured);
		double error = setpoint - measured; // difference between setpoint and measured
		integral += error * dt / 1000.0; // number of units times the number of seconds between updates, is accumulated over time
		double derivative = (error - previous_error) / dt;
		double output = (P / 1000.0) * error + (I / 1000.0) * integral + (D / 1000.0) * derivative;

		if (isLoad() && inDeadbandLoad()) {
			output = 0;
			integral = 0;
		}
		else if (isShoot() && inDeadbandShoot()) {
			output = 0;
			integral = 0;
		}
		else if (isOut() && inDeadbandOut()) {
			output = 0;
			integral = 0;
		}
		
		if (isOut())
			SmartDashboard.putNumber("Actual Out Posistion", pot.getValue());

		motor.set(limit(output));

		SmartDashboard.putNumber("Catapult Auto", Auto);
		SmartDashboard.putNumber("Catapult Current", motor.getOutputCurrent());
		SmartDashboard.putNumber("Catapult Setpoint", setpoint);
		SmartDashboard.putNumber("Catapult Output", limit(output));
		SmartDashboard.putNumber("Catapult Error (P)", error);
		SmartDashboard.putNumber("Catapult Integral (I)", integral);
		SmartDashboard.putNumber("Catapult Derivative (D)", derivative);

		previous_error = error;
		time = System.currentTimeMillis();
	}

	// public void calibrate(double input) { motor.set(input); SmartDashboard.putNumber("Catapult Output", input); SmartDashboard.putNumber("Catapult
	// Pot", pot.getValue()); }

	public void setLoad() {
		setpoint = Load;
		run();
	}

	public boolean isLoad() {
		return setpoint == Load;
	}

	public void setOut() {
		setpoint = Out;
		run();
	}

	public boolean isOut() {
		return setpoint == Out;
	}

	public void setShoot() {
		setpoint = Shoot;
		run();
	}

	public boolean isShoot() {
		return setpoint == Shoot;
	}

	public void setFar() {
		Shoot = Far;
	}

	public boolean isFar() {
		return Shoot == Far;
	}

	public void setClose() {
		Shoot = Close;
	}

	public boolean isClose() {
		return Shoot == Close;
	}

	public void setAuto(int s) {
		Shoot = s;
		Auto = s;
	}

	public boolean isAuto() {
		return Shoot == Auto;
	}

	public void latch() {
		latch.set(LATCHED);
		SmartDashboard.putBoolean("Latched", true);
		SmartDashboard.putBoolean("Released", false);
	}

	public void release() {
		latch.set(RELEASE);
		SmartDashboard.putBoolean("Latched", false);
		SmartDashboard.putBoolean("Released", true);
	}

	public double limit(double in) {
		if (in > MAX)
			return MAX;
		else if (in < -MAX)
			return -MAX;
		return in;
	}

	public boolean inDeadbandOut() {
		return Math.abs(Out - pot.getValue()) < RADIUS;
	}

	public boolean inDeadbandFar() {
		return Math.abs(Far - pot.getValue()) < RADIUS;
	}

	public boolean inDeadbandClose() {
		return Math.abs(Close - pot.getValue()) < RADIUS;
	}

	public boolean inDeadbandAuto() {
		return Math.abs(Auto - pot.getValue()) < RADIUS;
	}

	public boolean inDeadbandLoad() {
		return Math.abs(Load - pot.getValue()) < RADIUS;
	}

	public boolean inDeadbandShoot() {
		return Math.abs(Shoot - pot.getValue()) < RADIUS;
	}

	public boolean cocked() {
		return inDeadbandShoot() && latch.get() == LATCHED;
	}

	public double getPos() {
		return pot.getValue();
	}

	public void testCatapultActuator(double speed) {
		motor.set(speed);
		SmartDashboard.putNumber("Catapult Pot", pot.getValue());
	}
}
