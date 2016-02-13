package org.usfirst.frc.team1732.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends SampleRobot {

	final int STICK_LEFT_VERT = 1;
	final int STICK_LEFT_HORI = 2;
	final int STICK_RIGHT_VERT = 3;
	final int STICK_TIGHT_HORT = 4;
	
	final double MOTOR_MIN = 0.1;
	
	final double ARM_POS_TOP = 45; 	// degrees
	final double ARM_POS_BOT =  0; 	// degrees
	
	final int BUTTON_A = 0;
	final int BUTTON_B = 1;
	final int BUTTON_X = 2;
	final int BUTTON_Y = 3;
	final int BUTTON_R = 4;
	final int BUTTON_L = 5;


	CANTalon left_1;
	CANTalon left_2;
	CANTalon left_3;
	Encoder left_encoder;

	CANTalon right_1;
	CANTalon right_2;
	CANTalon right_3;
	Encoder right_encoder;

	CANTalon intakarino;
	Solenoid intakamatic;

	Solenoid magic_fingers;
	CANTalon cocker;
	Solenoid firer;

	CANTalon extendo;
	Encoder extendo_encoder;
	DigitalInput extendo_top;
	DigitalInput extendo_bot;

	CANTalon that_thing_that_hangs_1;
	CANTalon that_thing_that_hangs_2;
	Encoder that_thing_that_hangs_encoder;

	CANTalon defensomatic;
	
	Joystick controller;

	public Robot() {
		left_1 = new CANTalon(0);
		left_2 = new CANTalon(1);
		left_3 = new CANTalon(2);
		right_1 = new CANTalon(3);
		right_2 = new CANTalon(4);
		right_3 = new CANTalon(6);

		intakarino = new CANTalon(7);

		magic_fingers = new Solenoid(20, 0);
		cocker = new CANTalon(8);
		firer = new Solenoid(20, 1);

		extendo = new CANTalon(9);
		extendo_encoder = new Encoder(0, 1);
		extendo_top = new DigitalInput(2);
		extendo_bot = new DigitalInput(3);
		
		that_thing_that_hangs_1 = new CANTalon(10);
		that_thing_that_hangs_2 = new CANTalon(10);

		defensomatic = new CANTalon(11);

		controller = new Joystick(0);

	}

	public void robotMain() {
		while (true) {
			while (isEnabled()) {
				while (isAutonomous()) {
					System.out.println("Auto Running!");
				}
				while (isOperatorControl()) {
					setMotors(controller.getRawAxis(STICK_LEFT_VERT), controller.getRawAxis(STICK_RIGHT_VERT));
				}
				if (!isAutonomous() && !isOperatorControl())
					System.err.println("Enabled, but NOT Auto or Telep! Baka!");
			}
			while (isDisabled()) {
				left_1.set(0);
				left_2.set(0);
				left_3.set(0);
				right_1.set(0);
				right_2.set(0);
				right_3.set(0);

				intakarino.set(0);
				cocker.set(0);
				extendo.set(0);
				that_thing_that_hangs_1.set(0);
				that_thing_that_hangs_2.set(0);
				defensomatic.set(0);
			}
			if (!isEnabled() && !isDisabled())
				System.err.println("Not Enabled or Disabled? Coma? FUCK! What happened?");
		}
	}

	private void setMotors(double left, double right) {
		left_1.set(left);
		left_2.set(left);
		left_3.set(left);
		right_1.set(-right);
		right_2.set(-right);
		right_3.set(-right);
	}

	/*
	 * String autoSelected = (String) chooser.getSelected();
	 * 
	 * SendableChooser chooser;
	 * 
	 * final String defaultAuto = "Default"; final String auto1 = "My Auto 1";
	 * final String auto2 = "My Auto 2"; final String auto3 = "My Auto 3"; final
	 * String auto4 = "My Auto 4"; final String auto5 = "My Auto 5"; final
	 * String auto6 = "My Auto 6";
	 * 
	 * chooser = new SendableChooser(); chooser.addDefault("Default Auto",
	 * defaultAuto); chooser.addObject("My Auto 1", auto1); chooser.addObject(
	 * "My Auto 2", auto2); chooser.addObject("My Auto 3", auto3);
	 * chooser.addObject("My Auto 4", auto4); chooser.addObject("My Auto 5",
	 * auto5); chooser.addObject("My Auto 6", auto6); SmartDashboard.putData(
	 * "Auto modes", chooser);
	 */
}
