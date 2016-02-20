package org.usfirst.frc.team1732.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Solenoid;

public class Catapult {
	private Solenoid latch;
	private CANTalon reload;
	private AnalogInput pot;
	private Fingers fingers;
	
	private static final double RELOAD_SPEED = 0.5;
	private static final double RETURN_SPEED = -0.5;
	private static final double STOP = 0;
	
	private static final boolean LATCH_OPEN = true;
	private static final boolean LATCH_CLOSED = false;
	
	private static final int RELOAD = 100;
	private static final int RETURN = 0;
	
	private long start;
	private STATES state;
	
	private enum STATES {
		WAIT_FOR_SHOOT_SIGNAL,
		OPEN_FINGERS_TO_SHOOT,
		UNLATCH_TO_SHOOT,
		MOTOR_RETURN,
		CLOSE_LATCH_TO_RELOAD,
		MOTOR_RELOAD,
		WAIT_FOR_BALL_SIGNAL,
		CLOSE_FINGERS_TO_SECURE,
		UNKNOWN
	}
	
	public Catapult() {
		latch = new Solenoid(2, 4);
		reload = new CANTalon(11);
		pot = new AnalogInput(1);
		fingers = new Fingers();
		state = STATES.UNKNOWN;
		start = System.currentTimeMillis();
	}
	
	public void shoot(boolean shoot, boolean ball) {
		if (state == STATES.WAIT_FOR_SHOOT_SIGNAL) {
			if (shoot) {
				start = System.currentTimeMillis();
				state = STATES.OPEN_FINGERS_TO_SHOOT;
			}
		} else if (state == STATES.OPEN_FINGERS_TO_SHOOT) {
			fingers.open();
			if (System.currentTimeMillis() - start > 50) {
				start = System.currentTimeMillis();
				state = STATES.UNLATCH_TO_SHOOT;
			}
		} else if (state == STATES.UNLATCH_TO_SHOOT) {
			latch.set(LATCH_OPEN);
			if (System.currentTimeMillis() - start > 50) {
				state = STATES.MOTOR_RETURN;
			}
		} else if (state == STATES.MOTOR_RETURN) {
			reload.set(RETURN_SPEED); // TODO: write pid
			if (Math.abs(pot.getValue() - RETURN) < 5) {
				reload.set(STOP);
				start = System.currentTimeMillis();
				state = STATES.CLOSE_LATCH_TO_RELOAD;
			}
		} else if (state == STATES.CLOSE_LATCH_TO_RELOAD) {
			latch.set(LATCH_CLOSED);
			if (System.currentTimeMillis() - start > 50) {
				state = STATES.MOTOR_RELOAD;
			}
		} else if (state == STATES.MOTOR_RELOAD) {
			reload.set(RELOAD_SPEED); // TODO: write pid
			if (Math.abs(pot.getValue() - RELOAD) < 5) {
				reload.set(STOP);
				state = STATES.WAIT_FOR_BALL_SIGNAL;
			}
		} else if (state == STATES.WAIT_FOR_BALL_SIGNAL) {
			if (ball) {
				start = System.currentTimeMillis();
				state = STATES.CLOSE_FINGERS_TO_SECURE;
			}
		} else if (state == STATES.CLOSE_FINGERS_TO_SECURE) {
			fingers.close();
			if (System.currentTimeMillis() - start > 50) {
				state = STATES.WAIT_FOR_SHOOT_SIGNAL;
			}
		} else if (state == STATES.UNKNOWN) {
			if (latch.get() == LATCH_OPEN) {
				state = STATES.MOTOR_RETURN;
				fingers.open(); // may need to be removed depending on starting position
			} else if (latch.get() == LATCH_CLOSED) {
				state = STATES.MOTOR_RELOAD;
				fingers.open(); // may need to be removed depending on starting position
			} else {
				System.err.println("Unknown State");
			}
		} else {
			System.err.println("State Machine Broken");
		}
	}
	
	public void disable() {
		reload.set(STOP);
	}
}
