package org.usfirst.frc.team1732.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm {
	private CANTalon motor = new CANTalon(17);
	private AnalogInput pot = new AnalogInput(2); //3 on practice, 2 on competition bot

	private double previous_error = 0;
	private double previous = 0;
	private double integral = 0;
	private long time = System.currentTimeMillis();

	private double P = 4.0;
	private double I = 0.0;
	private double D = 0.0;
	private double MAX = 0.8;
	private double MIN = 0.2;
	private double RAMP = 0.5;

	private int setpoint = High;
	//private int dir = 0;

	private static final int RADIUS = 50;
	private static final double UP_SPEED = 0.4;
	private static final double DOWN_SPEED = -0.4;
	private static final double STOP = 0;

	static int Low = (735); // 640 = practice, 735 = competition
	static int Middle = (1450); // 1450 practice and competition
	static int Auto = (3120+1450)/2;
	static int High = (3340); // 3135 practice, 3340 competition, 3120 was old competition
	
	//private boolean auto = false;

	public Arm() {
		SmartDashboard.putNumber("Arm P", P);
		SmartDashboard.putNumber("Arm Low", Low);
		SmartDashboard.putNumber("Arm Middle", Middle);
		SmartDashboard.putNumber("Arm High", High);
		SmartDashboard.putNumber("Arm Auto", Auto);
		SmartDashboard.putNumber("Arm I", I);
		SmartDashboard.putNumber("Arm D", D);
		SmartDashboard.putNumber("Arm MAX", MAX);
		SmartDashboard.putNumber("Arm RAMP", RAMP);
		if (inDeadbandLow()) setpoint = Low;
		else if (inDeadbandMiddle()) setpoint = Middle;
		else if (inDeadbandHigh()) setpoint = High;
		else if(inDeadbandAuto()) setpoint = Auto;
	}

	public void run() {
		/*if (!auto) {
			if (dir == -1) {
				motor.set(DOWN_SPEED);
			} else if (dir == 1) {
				motor.set(UP_SPEED);
			} else if (dir == 0) {
				motor.set(STOP);
			} else {
				System.err.println("Unknown Arm Direction");
				motor.set(STOP);
			}
		} else {*/
			P = SmartDashboard.getNumber("Arm P", P);
			I = SmartDashboard.getNumber("Arm I", I);
			D = SmartDashboard.getNumber("Arm D", D);
			MAX = SmartDashboard.getNumber("Arm MAX", MAX);
			RAMP = SmartDashboard.getNumber("Arm RAMP", RAMP);

			Low = (int) SmartDashboard.getNumber("Arm Low", Low);
			Middle = (int) SmartDashboard.getNumber("Arm Middle", Middle);
			High = (int) SmartDashboard.getNumber("Arm High", High);
			Auto = (int) SmartDashboard.getNumber("Arm Auto", Auto);

			double measured = pot.getValue();
			SmartDashboard.putNumber("Arm Pot", measured);
			double dt = (System.currentTimeMillis() - time);
			double error = setpoint - measured;
			integral += error * dt / 1000.0;
			double derivative = (error - previous_error) / dt;
			double output = (P / 1000.0) * error + (I / 1000.0) * integral + (D / 1000.0) * derivative;

			if (output < 1) {
				if (previous - output > RAMP / 1000 * dt) {
					output = previous - (RAMP / 1000 * dt);
				}
			} else {
				if (output - previous > RAMP / 1000 * dt) {
					output = previous + (RAMP / 1000 * dt);
				}
			}

			if (isHigh() && inDeadbandHigh())
				output = 0;
			else if (isMiddle() && inDeadbandMiddle())
				output = 0;
			else if (isLow() && inDeadbandLow())
				output = 0;
			else if(isAuto() && inDeadbandAuto())
				output = 0;
			
			motor.set(limit(limit_low(output)));

			SmartDashboard.putNumber("Arm Setpoint", setpoint);
			SmartDashboard.putNumber("Arm Output", limit(limit_low(output)));
			SmartDashboard.putNumber("Arm Error (P)", error);
			SmartDashboard.putNumber("Arm Integral (I)", integral);
			SmartDashboard.putNumber("Arm Derivative (D)", derivative);

			previous = output;
			previous_error = error;
			time = System.currentTimeMillis();
		//}
	}
	/*public void setUp() {
		auto = false;
		dir = 1;
	}

	public void setDown() {
		auto = false;
		dir = -1;
	}
	
	public void stop() {
		auto = false;
		dir = 0;
	}*/

	public void setLow() {
		//auto = true;
		setpoint = Low;
		run();
		//motor.set(DOWN_SPEED);
	}

	public boolean isLow() {
		return setpoint == Low;
	}

	public void setMiddle() {
		//auto = true;
		setpoint = Middle;
		run();
		//motor.set(STOP);
	}
	public void setAuto(){
		setpoint = Auto;
		run();
	}
	public boolean isAuto(){
		return setpoint == Auto;
	}
	public boolean isMiddle() {
		return setpoint == Middle;
	}

	public void setHigh() {
		//auto = true;
		setpoint = High;
		run();
		//motor.set(UP_SPEED);
	}

	public boolean isHigh() {
		return setpoint == High;
	}

	//public boolean isAuto() {
	//	return auto;
	//}
	
	public double limit_low(double in) {
		if (in < MIN && in > -1 * MIN)
			return 0;
		else
			return in;
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
	
	public boolean inDeadbandAuto(){
		return Math.abs(Auto - pot.getValue()) < RADIUS;
	}
	
	public boolean inDeadbandMiddle() {
		return Math.abs(Middle - pot.getValue()) < 2 * RADIUS;
	}

	public boolean inDeadbandLow() {
		return Math.abs(Low - pot.getValue()) < RADIUS;
	}

	public double getPos() {
		return pot.getValue();
	}
	
	public void testArmActuator(double speed) {
		motor.set(speed);
		SmartDashboard.putNumber("Arm Pot", pot.getValue());
	}
}