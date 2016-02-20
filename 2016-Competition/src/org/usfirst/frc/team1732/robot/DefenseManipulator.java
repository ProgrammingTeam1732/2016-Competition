package org.usfirst.frc.team1732.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;

public class DefenseManipulator {
	private CANTalon manipulate;
	private DigitalInput limit;
	
	private static final double STOP = 0;
	
	public DefenseManipulator() {
		manipulate = new CANTalon(18);
		limit = new DigitalInput(0);
	}
	
	public void defensomatic() {
		return;
	}
	
	public void disable() {
		manipulate.set(STOP);
	}
}
