package org.usfirst.frc.team1732.robot;

import edu.wpi.first.wpilibj.Joystick;

public class Input {
	private Joystick left = new Joystick(0);
	private Joystick right = new Joystick(1);
	private Joystick controller = new Joystick(2);

	private final int STICK_VERT = 1;
	private final int STICK_HORI = 0;
	
	private final int CONTROL_LEFT_VERT = 1;
	private final int CONTROL_LEFT_HORI = 0;
	private final int CONTROL_RIGHT_VERT = 3;
	private final int CONTROL_RIGHT_HORI = 2;
			
	private final int BUTTON_A = 2;
	private final int BUTTON_B = 3;
	private final int BUTTON_X = 1;
	private final int BUTTON_Y = 4;
	private final int BUTTON_LB = 5;
	private final int BUTTON_LT = 7;
	private final int BUTTON_RB = 6;
	private final int BUTTON_RT = 8;
	
	public boolean getA()	{ return controller.getRawButton(BUTTON_A); }
	public boolean getB()	{ return controller.getRawButton(BUTTON_B); }
	public boolean getX()	{ return controller.getRawButton(BUTTON_X); }
	public boolean getY()	{ return controller.getRawButton(BUTTON_Y); }
	public boolean getLB()	{ return controller.getRawButton(BUTTON_LB); }
	public boolean getLT()	{ return controller.getRawButton(BUTTON_LT); }
	public boolean getRB()	{ return controller.getRawButton(BUTTON_RB); }
	public boolean getRT()	{ return controller.getRawButton(BUTTON_RT); }
	
	public double getLeftVertC()	{ return controller.getRawAxis(CONTROL_LEFT_VERT); }
	public double getLeftHoriC()	{ return controller.getRawAxis(CONTROL_LEFT_HORI); }
	public double getRightVertC()	{ return controller.getRawAxis(CONTROL_RIGHT_VERT); }
	public double getRightHoriC()	{ return controller.getRawAxis(CONTROL_RIGHT_HORI); }
	
	public double getLeftVertJ()	{ return left.getRawAxis(STICK_VERT); }
	public double getLeftHoriJ()	{ return left.getRawAxis(STICK_HORI); }
	public double getRightVertJ()	{ return right.getRawAxis(STICK_VERT); }
	public double getRightHoriJ()	{ return right.getRawAxis(STICK_HORI); }

}
