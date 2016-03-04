package org.usfirst.frc.team1732.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive {
	private CANTalon left_1 = new CANTalon(14);
	private CANTalon left_2 = new CANTalon(13);
	private CANTalon left_3 = new CANTalon(15);
	private Encoder left_encoder = new Encoder(2, 3);

	private CANTalon right_1 = new CANTalon(20);
	private CANTalon right_2 = new CANTalon(19);
	private CANTalon right_3 = new CANTalon(21);
	private Encoder right_encoder = new Encoder(0, 1);
	
	public void drive(double left, double right) {
		SmartDashboard.putNumber("Drive Left", left);
		SmartDashboard.putNumber("Drive Right", right);
		//left = 0;
		//right = 0;
		left *= -1;
		left_1.set(left);
		left_2.set(left);
		left_3.set(left);
		right_1.set(right);
		right_2.set(right);
		right_3.set(right);
		
		SmartDashboard.putNumber("Encoder Left", left_encoder.get());
		SmartDashboard.putNumber("Encoder Right", right_encoder.get());
	}
	
	private int left = 0;
	private int right = 0;
	
	public void reset() {
		left = left_encoder.get();
		right = right_encoder.get();
	}
	
	public int getLeft() {
		return left_encoder.get() - left;
	}
	
	public int getRight() {
		return right_encoder.get() - right;
	}
}
