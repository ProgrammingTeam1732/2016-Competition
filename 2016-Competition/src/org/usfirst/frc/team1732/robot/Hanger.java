package org.usfirst.frc.team1732.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;

public class Hanger {
	private CANTalon left;
	private CANTalon right;
	private AnalogInput pot;
	
	private final static int BOT = 0;
	private final static int TOP = 100;
	
	private final static double LEFT_ROTATE = 0.5;
	private final static double RIGHT_ROTATE = -0.5;
	
	private final static double LEFT_LOWER = -0.3;
	private final static double RIGHT_LOWER = 0.3;
	
	private final static double LEFT_RAISE = 0.4;
	private final static double RIGHT_RAISE = -0.4;
	
	private final static double STOP = 0;
	
	public Hanger() {
		left = new CANTalon(16);
		right = new CANTalon(17);
		pot = new AnalogInput(0);
	}
	
	public void rotate() {
		if (pot.getValue() < TOP) {
			left.set(LEFT_ROTATE);
			right.set(RIGHT_ROTATE);
		}
	}
	
	public void disable() {
		left.set(STOP);
		right.set(STOP);
	}
}
