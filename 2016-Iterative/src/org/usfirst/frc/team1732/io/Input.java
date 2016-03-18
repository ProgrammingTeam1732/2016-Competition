package org.usfirst.frc.team1732.io;

import edu.wpi.first.wpilibj.Joystick;

public class Input {
	private Joystick left = new Joystick(0);
	private Joystick right = new Joystick(1);
	//private Joystick controller = new Joystick(2);

	private Joystick button1 = new Joystick(2);
	private Joystick button2 = new Joystick(3);
	
	private final int STICK_VERT = 1;
	private final int STICK_HORI = 0;
	
	public boolean getManipulatorUp() { return button2.getRawButton(11); }
	public boolean getManipulatorNot() { return !getManipulatorUp() && !getManipulatorDown(); }
	public boolean getManipulatorDown() { return button2.getRawButton(12); }
	
	public boolean getArmHigh() { return button1.getRawButton(9); }
	public boolean getArmMiddle() { return button1.getRawButton(8); }
	public boolean getArmLow() { return button1.getRawButton(10); }
	
	//public boolean getArmUp() { return button2.getRawButton(3); }
	//public boolean getArmNot() { return !getArmUp() && !getArmDown(); }
	//public boolean getArmDown() { return button2.getRawButton(2); }
	
	public boolean getShoot() { return  button2.getRawButton(1); }
	
	public boolean getClimberUp() { return button1.getRawButton(11); }
	public boolean getClimberDown() { return button1.getRawButton(12); }
	
	public boolean getFingersOpen() { return button1.getRawButton(5); }
	public boolean getFingersNot() { return !getFingersOpen() && !getFingersClose(); }
	public boolean getFingersClose() { return button1.getRawButton(6); }

	public boolean getIntakeUp() { return button1.getRawButton(2); }
	public boolean getIntakeNot() { return !getIntakeUp() && !getIntakeDown(); }
	public boolean getIntakeDown() { return button1.getRawButton(1); }
	
	public boolean getIntakeIn() { return button1.getRawButton(3); }
	public boolean getIntakeOut() { return button1.getRawButton(4); }
	
	/*private final int CONTROL_LEFT_VERT = 1;
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
	private final int BUTTON_RS = 12;
	private final int BUTTON_LS = 11;
	private final int BUTTON_SELECT = 9;
	private final int BUTTON_START = 10;
	
	public boolean getA()	{ return controller.getRawButton(BUTTON_A); }
	public boolean getB()	{ return controller.getRawButton(BUTTON_B); }
	public boolean getX()	{ return controller.getRawButton(BUTTON_X); }
	public boolean getY()	{ return controller.getRawButton(BUTTON_Y); }
	public boolean getLB()	{ return controller.getRawButton(BUTTON_LB); }
	public boolean getLT()	{ return controller.getRawButton(BUTTON_LT); }
	public boolean getRB()	{ return controller.getRawButton(BUTTON_RB); }
	public boolean getRT()	{ return controller.getRawButton(BUTTON_RT); }
	public boolean getSTART()	{ return controller.getRawButton(BUTTON_START); }
	public boolean getSELECT()	{ return controller.getRawButton(BUTTON_SELECT); }
	public boolean getRS()	{ return controller.getRawButton(BUTTON_RS); }
	public boolean getLS()	{ return controller.getRawButton(BUTTON_LS); }

	
	public double getLeftVertC()	{ return controller.getRawAxis(CONTROL_LEFT_VERT); }
	public double getLeftHoriC()	{ return controller.getRawAxis(CONTROL_LEFT_HORI); }
	public double getRightVertC()	{ return controller.getRawAxis(CONTROL_RIGHT_VERT); }
	public double getRightHoriC()	{ return controller.getRawAxis(CONTROL_RIGHT_HORI); } */
	
	public double getLeftVert()	{ return left.getRawAxis(STICK_VERT); }
	public double getLeftHori()	{ return left.getRawAxis(STICK_HORI); }
	public double getRightVert()	{ return right.getRawAxis(STICK_VERT); }
	public double getRightHori()	{ return right.getRawAxis(STICK_HORI); }

}
