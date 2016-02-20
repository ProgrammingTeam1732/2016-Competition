package org.usfirst.frc.team1732.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends SampleRobot {

	final int STICK_VERT = 1;
	final int STICK_HORI = 0;
	
	final int CONTROL_LEFT_VERT = 1;
	final int CONTROL_LEFT_HORI = 0;
	final int CONTROL_RIGHT_VERT = 3;
	final int CONTROL_RIGHT_HORI = 2;
			
	final int BUTTON_A = 2;
	final int BUTTON_B = 3;
	final int BUTTON_X = 1;
	final int BUTTON_Y = 4;
	final int BUTTON_LB = 5;
	final int BUTTON_LT = 7;
	final int BUTTON_RB = 6;
	final int BUTTON_RT = 8;

	Drive drive;
	Arm arm;
	DefenseManipulator defense;
	Hanger hanger;
	Intake intake;
	
	Joystick left;
	Joystick right;
	Joystick controller;

	public Robot() {
		drive = new Drive();
		arm = new Arm();
		defense = new DefenseManipulator();
		hanger = new Hanger();
		intake = new Intake();
		
		left = new Joystick(0);
		right = new Joystick(1);
		controller = new Joystick(2);
		
		SmartDashboard.putBoolean("Drive Enabled", false);
		SmartDashboard.putBoolean("Arm Enabled", false);
		SmartDashboard.putBoolean("Defense Manipulator Enabled", false);
		SmartDashboard.putBoolean("Intake Enabled", false);
		SmartDashboard.putBoolean("Hanger Enabled", false);
	}

	public void robotMain() {
		while (true) {
			while (isEnabled()) {
				while (isAutonomous()) {
					System.out.println("Auto Running!");
				}
				while (isOperatorControl()) {
					if (SmartDashboard.getBoolean("Drive Enabled", false)) {
						drive.drive(left.getRawAxis(STICK_VERT), right.getRawAxis(STICK_VERT));
					}
					if (SmartDashboard.getBoolean("Arm Enabled", false)) {
					}
					if (SmartDashboard.getBoolean("Defense Manipulator Enabled", false)) {
					}
					if (SmartDashboard.getBoolean("Intake Enabled", false)) {
						if (controller.getRawButton(BUTTON_A)) {
							intake.setDown();
						} else if (controller.getRawButton(BUTTON_Y)) {
							intake.setUp();
						}
						
						if (controller.getRawButton(BUTTON_X)) {
							intake.setIn();
						} else if (controller.getRawButton(BUTTON_B)) {
							intake.setOut();
						} else {
							intake.setStop();
						}
					}
					if (SmartDashboard.getBoolean("Hanger Enabled", false)) {
						// do nothing
					}
				}
				if (!isAutonomous() && !isOperatorControl())
					System.err.println("Enabled, but NOT Auto or Telep! Baka!");
			}
			while (isDisabled()) {
				drive.disable();
				arm.disable();
				defense.disable();
				intake.disable();
				hanger.disable();
			}
			if (!isEnabled() && !isDisabled())
				System.err.println("Not Enabled or Disabled? Coma? FUCK! What happened?");
		}
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
