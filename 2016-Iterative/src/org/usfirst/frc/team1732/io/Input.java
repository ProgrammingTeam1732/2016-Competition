package org.usfirst.frc.team1732.io;

import edu.wpi.first.wpilibj.Joystick;

public class Input {
	private Joystick left = new Joystick(0);
	private Joystick right = new Joystick(1);
	//private Joystick controller = new Joystick(2);

	private Joystick button1 = new Joystick(2);
	
	private final int STICK_VERT = 1;
	private final int STICK_HORI = 0;
	private final int STICK_TRIGGER = 1;
	private final int STICK_TWO = 2;
	private final int STICK_THREE = 3;
	private final int STICK_FOUR = 4;
	private final int STICK_FIVE = 5;
	private final int STICK_SIX = 6;
	
	private final int MANIP_UP = 8;
	private final int MANIP_DOWN = 7;
	private final int ARM_HIGH = 4;
	private final int ARM_MID = 5;
	private final int ARM_LOW = 6;
	private final int SET_SHOOT_FAR = 2;
	private final int SET_SHOOT_CLOSE = 3;
	private final int SHOOT = 1;
	//private final int CLIMBER_UP = 11;
	//private final int CLIMBER_DOWN = 12;
	private final int FINGER = 12;
	private final int INTAKE = 9;
	private final int INTAKE_IN = 10;
	private final int INTAKE_OUT = 11;
	
	public boolean getManipulatorUp() { return button1.getRawButton(MANIP_UP); }
	public boolean getManipulatorDown() { return button1.getRawButton(MANIP_DOWN); }
	
	public boolean getArmHigh() { return button1.getRawButton(ARM_HIGH); }
	public boolean getArmMiddle() { return button1.getRawButton(ARM_MID); }
	public boolean getArmLow() { return button1.getRawButton(ARM_LOW); }
	
	public boolean getSetShootFar() { return button1.getRawButton(SET_SHOOT_FAR); }
	public boolean getSetShootAuto() { return !getSetShootFar() && !getSetShootClose(); }
	public boolean getSetShootClose() { return button1.getRawButton(SET_SHOOT_CLOSE); }
	
	public boolean getShoot() { return  button1.getRawButton(SHOOT) || left.getRawButton(STICK_TWO) || right.getRawButton(STICK_TWO); }
	
	//public boolean getClimberUp() { return button1.getRawButton(CLIMBER_UP); }
	//public boolean getClimberDown() { return button1.getRawButton(CLIMBER_DOWN); }
	
	public boolean getFingersOpen() { return button1.getRawButton(FINGER) || left.getRawButton(STICK_TRIGGER) || right.getRawButton(STICK_TRIGGER); }
	public boolean getFingersClose() { return !button1.getRawButton(FINGER); }

	public boolean getResetShot() {
		if(button1.getRawAxis(STICK_HORI) < -0.1) return true;
		else return false;
	}
	
	public boolean getIntakeUp() { return button1.getRawButton(INTAKE); }
	public boolean getIntakeDown() { return !button1.getRawButton(INTAKE); }
	
	public boolean getIntakeIn() { return button1.getRawButton(INTAKE_IN); }
	public boolean getIntakeOut() { return button1.getRawButton(INTAKE_OUT); }
	
	public double getLeftVert()	{ return left.getRawAxis(STICK_VERT); }
	public double getLeftHori()	{ return left.getRawAxis(STICK_HORI); }
	public double getRightVert()	{ return right.getRawAxis(STICK_VERT); }
	public double getRightHori()	{ return right.getRawAxis(STICK_HORI); }
	
	public boolean getLeftTrigger() {return left.getRawButton(STICK_TRIGGER);}
	public boolean getLeftTwo() {return left.getRawButton(STICK_TWO);}
	public boolean getLeftThree() {return left.getRawButton(STICK_THREE);}
	public boolean getLeftFour() {return left.getRawButton(STICK_FOUR);}
	public boolean getLeftFive() {return left.getRawButton(STICK_FIVE);}
	public boolean getLeftSix() {return left.getRawButton(STICK_SIX);}
	public boolean getRightTrigger() {return left.getRawButton(STICK_TRIGGER);}
	public boolean getRightTwo() {return left.getRawButton(STICK_TWO);}
	public boolean getRightThree() {return left.getRawButton(STICK_THREE);}
	public boolean getRightFour() {return left.getRawButton(STICK_FOUR);}
	public boolean getRightFive() {return left.getRawButton(STICK_FIVE);}
	public boolean getRightSix() {return left.getRawButton(STICK_SIX);}
	
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

}
