package org.usfirst.frc.team1732.io;

import edu.wpi.first.wpilibj.Joystick;

public class Input {
	//private Joystick left = new Joystick(0);
	//private Joystick right = new Joystick(1);
	//private Joystick controller = new Joystick(2);
	//private Joystick controller = new Joystick(0);
	//private Joystick button1 = new Joystick(2);
	private Joystick button1 = new Joystick(0);
	//private final int STICK_VERT = 1;
	private final int STICK_HORI = 0;
	/*private final int STICK_TRIGGER = 1;
	private final int STICK_TWO = 2;
	private final int STICK_THREE = 3;
	private final int STICK_FOUR = 4;
	private final int STICK_FIVE = 5;
	private final int STICK_SIX = 6;*/
	
	//private final int MANIP_UP = 8;
	//private final int MANIP_DOWN = 7;
	/*private final int ARM_HIGH = 4;
	private final int ARM_MID = 5;
	private final int ARM_LOW = 6;*/
	private final int ARM_HIGH = 4;
	private final int ARM_MID = 3;
	private final int ARM_LOW = 2;
	//private final int SET_SHOOT_FAR = 9;
	//private final int SET_SHOOT_CLOSE = 10;
	private final int SHOOT = 1;
	//private final int CLIMBER_UP = 11;
	//private final int CLIMBER_DOWN = 12;
	private final int FINGER = 6;
	private final int INTAKE = 5;
	private final int INTAKE_IN = 3;
	private final int INTAKE_OUT = 3;
	private boolean fingersOn = false;
	private boolean RBPressed = true;
	private enum FingerState{
		open, closed;
	}
	private boolean intakeOn = false;
	private boolean LBPressed = true;
	private enum IntakeState{
		open, closed;
	}
	public boolean getManipulatorUp() { return false;}//button1.getRawButton(MANIP_UP); }
	public boolean getManipulatorDown() { return false;}//button1.getRawButton(MANIP_DOWN); }
	
	public boolean getArmHigh() { return button1.getRawButton(ARM_HIGH); }
	public boolean getArmMiddle() { return button1.getRawButton(ARM_MID); }
	public boolean getArmCheval() { return button1.getRawAxis(STICK_HORI) > 0.1 ? true : false; }
	public boolean getArmLow() { return button1.getRawButton(ARM_LOW); }
	
	public boolean getSetShootFar() { return false;}//button1.getRawButton(SET_SHOOT_FAR); }
	public boolean getSetShootAuto() { return !getSetShootFar() && !getSetShootClose(); }
	public boolean getSetShootClose() { return true;}//button1.getRawButton(SET_SHOOT_CLOSE); }
	
	public boolean getShoot() { return button1.getRawButton(SHOOT); }//System.out.println("checking shoot");return  button1.getRawButton(SHOOT) || getLeftTwo() || getRightTwo(); }
	private FingerState previous = FingerState.closed;
	private FingerState present = FingerState.closed;
	//public boolean getClimberUp() { return button1.getRawButton(CLIMBER_UP); }
	//public boolean getClimberDown() { return button1.getRawButton(CLIMBER_DOWN); }
	
	public boolean getFingersOpen() {
		//return button1.getRawButton(FINGER);
		if(button1.getRawButton(FINGER) && !RBPressed) {
			RBPressed = true;
			fingersOn = !fingersOn;
		} else if (button1.getRawButton(FINGER) && RBPressed){
		} else {
			RBPressed = false;
		}
		return !fingersOn;
//		if(button1.getRawButton(FINGER)) present = FingerState.open;
//		else present = FingerState.closed;
//		boolean b = !present.equals(previous);
//		previous = present;
//		return present.equals(FingerState.open);
		}//return (button1.getRawButton(FINGER)) || getTriggers();}
	public boolean getFingersClose() {return !getFingersOpen(); }
	public boolean getTriggers() {return getLeftTrigger() || getRightTrigger();}

	public boolean getResetShot() {
		// We had to put this button onto a port used for joysticks
		// that is why it is being used as a joystick
		if(button1.getRawAxis(STICK_HORI) < -0.1) return true;
		else return false;
	}
	
	public boolean getIntakeUp() { 
		if(button1.getRawButton(INTAKE) && !LBPressed) {
			LBPressed = true;
			intakeOn = !intakeOn;
		} else if (button1.getRawButton(INTAKE) && LBPressed){
		} else {
			LBPressed = false;
		}
		return !intakeOn;
	}
	public boolean getIntakeDown() { return !button1.getRawButton(INTAKE); }
	
	public boolean getIntakeIn() { return button1.getRawAxis(INTAKE_IN) > .75 ; }
	public boolean getIntakeOut() { return button1.getRawAxis(INTAKE_OUT) < 0; }
	
	public double getLeftVert()	{ return limit(button1.getRawAxis(1));}//limit(left.getRawAxis(STICK_VERT)); }
	public double getLeftHori()	{ return button1.getRawAxis(0); }
	public double getRightVert()	{ return limit(button1.getRawAxis(5));}//limit(right.getRawAxis(STICK_VERT)); }
	public double getRightHori()	{ return button1.getRawAxis(4); }
	
	/*public boolean getLeftTrigger() {return left.getRawButton(STICK_TRIGGER);}
	public boolean getLeftTwo() {return left.getRawButton(STICK_TWO);}
	public boolean getLeftThree() {return left.getRawButton(STICK_THREE);}
	public boolean getLeftFour() {return left.getRawButton(STICK_FOUR);}
	public boolean getLeftFive() {return left.getRawButton(STICK_FIVE);}
	public boolean getLeftSix() {return left.getRawButton(STICK_SIX);}
	public boolean getRightTrigger() {return right.getRawButton(STICK_TRIGGER);}
	public boolean getRightTwo() {return right.getRawButton(STICK_TWO);}
	public boolean getRightThree() {return right.getRawButton(STICK_THREE);}
	public boolean getRightFour() {return right.getRawButton(STICK_FOUR);}
	public boolean getRightFive() {return right.getRawButton(STICK_FIVE);}
	public boolean getRightSix() {return right.getRawButton(STICK_SIX);}*/
	public boolean getLeftTrigger() {return false;}
	public boolean getLeftTwo() {return false;}
	public boolean getLeftThree() {return false;}
	public boolean getLeftFour() {return false;}
	public boolean getLeftFive() {return false;}
	public boolean getLeftSix() {return false;}
	public boolean getRightTrigger() {return false;}
	public boolean getRightTwo() {return false;}
	public boolean getRightThree() {return false;}
	public boolean getRightFour() {return false;}
	public boolean getRightFive() {return false;}
	public boolean getRightSix() {return false;}
	
	public double limit(double d) {
		return Math.abs(d) < 0.1 ? 0.0 : d;
	}
	
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
