package org.usfirst.frc.team1732.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm {
	private CANTalon raise;
	private AnalogInput pot;
	private Catapult catapult;
	
	Position position;
	
	private final static int LOWERED = 0;
	private final static int TRAVERSAL = 50;
	private final static int RAISED = 100;
	
	public enum Position {
		LOW,
		TRAVEL,
		RAISED
	}
	
	private final static double ARM_UP = 0.5;
	private final static double ARM_DOWN = -0.5;
	private final static double STOP = 0;
	
	public Arm() {
		raise = new CANTalon(10);
		pot = new AnalogInput(0);
		catapult = new Catapult();
	}
	
	public void calibrate(int speed) {
		raise.set(speed);
		SmartDashboard.putNumber("Arm Speed", speed);
		SmartDashboard.putNumber("Arm Position", pot.getValue());
	}
	
	public void setPos(Position pos) {
		position = pos;
	}
	
	private void pid() {
		raise.set(STOP); // TODO Implement pid
	}
	
	public void shoot() {
		if (pot.getValue() < RAISED) {
			raise.set(ARM_UP);
			catapult.shoot(false, false);
		}
		else {
			raise.set(STOP);
			catapult.shoot(true, false);
		}
	}
	
	public void grab() {
		if (pot.getValue() > LOWERED) {
			raise.set(ARM_DOWN);
			catapult.shoot(false, false);
		}
		else {
			raise.set(STOP);
			catapult.shoot(false, true);
		}
	}
	
	public void travel() {
		if (pot.getValue() < TRAVERSAL) {
			raise.set(ARM_UP);
			catapult.shoot(false, false);
		} else if (pot.getValue() > TRAVERSAL) {
			raise.set(ARM_DOWN);
			catapult.shoot(false, false);
		} else if (pot.getValue() == TRAVERSAL) {
			raise.set(STOP);
			catapult.shoot(false, false);
		}
	}
	
	public void disable() {
		raise.set(STOP);
		catapult.disable();
	}
}
