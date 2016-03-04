package org.usfirst.frc.team1732.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DefenseManipulator {
	private CANTalon manipulate = new CANTalon(11);
	private Encoder pos = new Encoder(6, 7);
	
	public void setLow() {
		manipulate.set(0.3);
		SmartDashboard.putNumber("Denfense Manipulator Pos", pos.get());
	}
	
	public void setHigh() {
		manipulate.set(-0.3);
		SmartDashboard.putNumber("Denfense Manipulator Pos", pos.get());
	}
	
	public void stop() {
		manipulate.set(0);
		SmartDashboard.putNumber("Denfense Manipulator Pos", pos.get());		
	}
}
 