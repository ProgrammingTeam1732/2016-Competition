package org.usfirst.frc.team1732.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Encoder;

public class Drive {

	CANTalon left_1;
	CANTalon left_2;
	CANTalon left_3;
	Encoder left_encoder;

	CANTalon right_1;
	CANTalon right_2;
	CANTalon right_3;
	Encoder right_encoder;
	
	private static final double STOP = 0;
	
	public Drive() {
		left_1 = new CANTalon(14);
		left_2 = new CANTalon(13);
		left_3 = new CANTalon(15);
		right_1 = new CANTalon(20);
		right_2 = new CANTalon(19);
		right_3 = new CANTalon(21);
		
		left_encoder = new Encoder(0, 1);
		right_encoder = new Encoder(2, 3);
	}
	
	public void drive(double left, double right) {
		right *= -1;
		left_1.set(left);
		left_2.set(left);
		left_3.set(left);
		right_1.set(right);
		right_2.set(right);
		right_3.set(right);
	}
	
	public void disable() {
		drive(STOP, STOP);
	}
}
