package org.usfirst.frc.team1732.subsystems;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;

public class Hanger {
	@SuppressWarnings("unused") private CANTalon left = new CANTalon(16);
	@SuppressWarnings("unused") private CANTalon right = new CANTalon(10);
	@SuppressWarnings("unused") private Encoder pot = new Encoder(4, 5);
	@SuppressWarnings("unused") private Solenoid clamps = new Solenoid(2, 2);
}
