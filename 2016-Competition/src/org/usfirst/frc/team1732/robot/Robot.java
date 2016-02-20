package org.usfirst.frc.team1732.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends SampleRobot {

	final int STICK_VERT = 1;
	final int STICK_HORI = 2;

	final double MOTOR_MIN = 0.1;

	final double ARM_POS_TOP = 45; // degrees
	final double ARM_POS_BOT = 0; // degrees

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

	Joystick joystick_left;
	Joystick joystick_right;

	public Robot() {
		left_1 = new CANTalon(10);
		left_2 = new CANTalon(11);
		left_3 = new CANTalon(12);
		right_1 = new CANTalon(13);
		right_2 = new CANTalon(14);
		right_3 = new CANTalon(15);

		intakarino = new CANTalon(16);

		magic_fingers = new Solenoid(2, 0);
		cocker = new CANTalon(17);
		firer = new Solenoid(2, 1);

		extendo = new CANTalon(18);
		extendo_encoder = new Encoder(0, 1);
		extendo_top = new DigitalInput(2);
		extendo_bot = new DigitalInput(3);

		that_thing_that_hangs_1 = new CANTalon(19);
		that_thing_that_hangs_2 = new CANTalon(20);

		defensomatic = new CANTalon(21);

		joystick_left = new Joystick(0);
		joystick_right = new Joystick(1);

	}

	public void robotMain() {
		while (true) {
			while (isEnabled()) {
				while (isAutonomous()) {
					System.out.println("Auto Running!");
				}
				while (isOperatorControl()) {
					setMotors(joystick_left.getRawAxis(STICK_VERT), joystick_right.getRawAxis(STICK_VERT));

					}
					
				};
				
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

	private void intakarino(boolean a1) {
			if( a1 == true){
				intakarino.set(1);
			} else {
				intakarino.set(0); 
			}
	}
	private void intakamatic(boolean a3) {
		if( a3 == true){
			intakamatic.set(1);
		} else {
			intakamatic.set(0); 
		}
}
	
	private void magicfingers(boolean a2){
		if(a2 == true) {
				magic_fingers.set(true);
		}else {
				magic_fingers.set(false);
		}
	}
    public void cockPult(boolean cock/*hahahah stfu*/){
    	
    if (cock == true){ // If the button is pushed, the follwing while loop is executed
    	while( /*hall effect sensor #1*/ || /*hall effect sensor #2*/ != 1){ //If the hall sesnors are activated, the actuator is fully extended, so while the hall effect sensors ARE NOT 1, the motors cock the catapult back
    		cockPult.set(1);
    	} else {
    			cockPult.set(0);//When the hall effect sensors are 1, meaning the acutators are fully extedned, the motor is set to 0.
    		}
    	}
    }
>>>>>>> origin/master
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
