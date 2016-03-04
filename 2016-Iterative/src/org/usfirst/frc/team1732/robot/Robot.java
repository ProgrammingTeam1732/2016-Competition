
package org.usfirst.frc.team1732.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	final String defaultAuto = "Default";
	final String customAuto = "Miato";
	String autoSelected;
	SendableChooser chooser;

	// Encoder one = new Encoder(0, 1);
	// Encoder two = new Encoder(2, 3);
	// Encoder thr = new Encoder(4, 5);
	// Encoder fou = new Encoder(6, 7);
	
	
	Drive drive = new Drive();
	Intake intake = new Intake();
	Arm arm = new Arm();
	Catapult catapult = new Catapult();
	Fingers fingers = new Fingers();
	Input input = new Input();

	StateMachine sm = new StateMachine();
	
	StateMachine portcullis = new StateMachine();
	StateMachine lowbar = new StateMachine();
	StateMachine rough_terrain = new StateMachine();
	StateMachine draw_bridge = new StateMachine();
	StateMachine ramparts = new StateMachine();
	StateMachine moat = new StateMachine();
	StateMachine sally_port = new StateMachine();
	StateMachine cheval_de_frise = new StateMachine();
	StateMachine rock_wall = new StateMachine();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		chooser = new SendableChooser();
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto choices", chooser);
		
		sm.addState(new State((RobotState rbs) -> {
			return rbs.shoot;
		} , (RobotState rbs) -> {
			SmartDashboard.putString("State", "Waiting to shoot");
			return new RobotInstruction();
		})).addState(new State((RobotState rbs) -> {
			return (Math.abs(System.currentTimeMillis() - rbs.start_time) > 500);
		} , (RobotState rbs) -> {
			SmartDashboard.putString("State", "Opening Fingers");
			RobotInstruction rbi = new RobotInstruction();
			rbi.fingers_open = true;
			return rbi;
		})).addState(new State((RobotState rbs) -> {
			return Math.abs(System.currentTimeMillis() - rbs.start_time) > 500;
		} , (RobotState rbs) -> {
			SmartDashboard.putString("State", "Shooting");
			RobotInstruction rbi = new RobotInstruction();
			rbi.catapult_release = true;
			return rbi;
		})).addState(new State((RobotState rbs) -> {
			return rbs.catapult_aligned_out;
		} , (RobotState rbs) -> {
			SmartDashboard.putString("State", "Retrive Tram");
			RobotInstruction rbi = new RobotInstruction();
			rbi.catapult_out = true;
			return rbi;
		})).addState(new State((RobotState rbs) -> {
			return Math.abs(System.currentTimeMillis() - rbs.start_time) > 500;
		} , (RobotState rbs) -> {
			SmartDashboard.putString("State", "Latching Tram");
			RobotInstruction rbi = new RobotInstruction();
			rbi.catapult_latch = true;
			return rbi;
		})).addState(new State((RobotState rbs) -> {
			return rbs.catapult_aligned_in;
		} , (RobotState rbs) -> {
			SmartDashboard.putString("State", "Pulling Tram");
			RobotInstruction rbi = new RobotInstruction();
			rbi.catapult_in = true;
			return rbi;
		})).addState(new State((RobotState rbs) -> {
			return rbs.ball;
		} , (RobotState rbs) -> {
			SmartDashboard.putString("State", "Waiting for Ball");
			return new RobotInstruction();
		})).addState(new State((RobotState rbs) -> {
			return Math.abs(System.currentTimeMillis() - rbs.start_time) > 500;
		} , (RobotState rbs) -> {
			SmartDashboard.putString("State", "Closing Fingers");
			RobotInstruction rbi = new RobotInstruction();
			rbi.fingers_close = true;
			return rbi;
		}));
		
		
		rock_wall.addState(new State((RobotState rbs) -> {
			return ((System.currentTimeMillis() - rbs.start_time > 10000) || (rbs.drive_right_dist > 1000) || (rbs.drive_left_dist > 1000)) && !(System.currentTimeMillis() - rbs.start_time > 5000);
		} , (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.drive_left = 0.5;
			rbi.drive_right = 0.5;
			return rbi;
		})).addState(new State((RobotState rbs) -> {
			return true;
		} , (RobotState rbs) -> {
			RobotInstruction rbi =  new RobotInstruction();
			rbi.machine_finished = true;
			return rbi;
		}));
		
		lowbar.addState(new State((RobotState rbs) -> {
			return ((System.currentTimeMillis() - rbs.start_time > 10000) || (rbs.drive_right_dist > 1000) || (rbs.drive_left_dist > 1000)) && !(System.currentTimeMillis() - rbs.start_time > 5000);
		} , (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.drive_left = 0.5;
			rbi.drive_right = 0.5;
			return rbi;
		})).addState(new State((RobotState rbs) -> {
			return true;
		} , (RobotState rbs) -> {
			RobotInstruction rbi =  new RobotInstruction();
			rbi.machine_finished = true;
			return rbi;
		}));
		
		rough_terrain.addState(new State((RobotState rbs) -> {
			return ((System.currentTimeMillis() - rbs.start_time > 10000) || (rbs.drive_right_dist > 1000) || (rbs.drive_left_dist > 1000)) && !(System.currentTimeMillis() - rbs.start_time > 5000);
		} , (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.drive_left = 0.5;
			rbi.drive_right = 0.5;
			return rbi;
		})).addState(new State((RobotState rbs) -> {
			return true;
		} , (RobotState rbs) -> {
			RobotInstruction rbi =  new RobotInstruction();
			rbi.machine_finished = true;
			return rbi;
		}));
		
		moat.addState(new State((RobotState rbs) -> {
			return ((System.currentTimeMillis() - rbs.start_time > 10000) || (rbs.drive_right_dist > 1000) || (rbs.drive_left_dist > 1000)) && !(System.currentTimeMillis() - rbs.start_time > 5000);
		} , (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.drive_left = 0.5;
			rbi.drive_right = 0.5;
			return rbi;
		})).addState(new State((RobotState rbs) -> {
			return true;
		} , (RobotState rbs) -> {
			RobotInstruction rbi =  new RobotInstruction();
			rbi.machine_finished = true;
			return rbi;
		}));
		
		ramparts.addState(new State((RobotState rbs) -> {
			return ((System.currentTimeMillis() - rbs.start_time > 10000) || (rbs.drive_right_dist > 1000) || (rbs.drive_left_dist > 1000)) && !(System.currentTimeMillis() - rbs.start_time > 5000);
		} , (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.drive_left = 0.5;
			rbi.drive_right = 0.5;
			return rbi;
		})).addState(new State((RobotState rbs) -> {
			return true;
		} , (RobotState rbs) -> {
			RobotInstruction rbi =  new RobotInstruction();
			rbi.machine_finished = true;
			return rbi;
		}));
		
		rock_wall.addState(new State((RobotState rbs) -> {
			return ((System.currentTimeMillis() - rbs.start_time > 10000) || (rbs.drive_right_dist > 1000) || (rbs.drive_left_dist > 1000)) && !(System.currentTimeMillis() - rbs.start_time > 5000);
		} , (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.drive_left = 0.5;
			rbi.drive_right = 0.5;
			return rbi;
		})).addState(new State((RobotState rbs) -> {
			return true;
		} , (RobotState rbs) -> {
			RobotInstruction rbi =  new RobotInstruction();
			rbi.machine_finished = true;
			return rbi;
		}));
		
		
	}

	/*
	 * rock_wall.addState(new State((RobotState rbs) -> {
			
		} , (RobotState rbs) -> {
			
		}));
	 */
	
	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	public void autonomousInit() {
		autoSelected = (String) chooser.getSelected();
		System.out.println("Auto selected: " + autoSelected);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		switch (autoSelected) {
		case customAuto:
			// Put custom auto code here
			break;
		case defaultAuto:
		default:
			// Put default auto code here
			break;
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		
		// SmartDashboard.putNumber("One", one.get());
		// SmartDashboard.putNumber("Two", two.get());
		// SmartDashboard.putNumber("Thr", thr.get());
		// SmartDashboard.putNumber("Fou", fou.get());
		
		SmartDashboard.putNumber("Current Time", System.currentTimeMillis());
		
		drive.drive(input.getLeftVertC(), input.getRightVertC()); // drive
																	// always

		RobotState rbs = new RobotState();
		rbs.shoot = input.getB();
		rbs.ball = input.getRT();
		rbs.arm_aligned = arm.inDeadband();
		rbs.catapult_aligned_out = catapult.inDeadbandOut();
		rbs.catapult_aligned_in = catapult.inDeadbandIn();
		rbs.low = arm.isLow();
		rbs.middle = arm.isMiddle();
		rbs.high = arm.isHigh();
		rbs.drive_left_dist = drive.getLeft();
		rbs.drive_right_dist = drive.getRight();

		RobotInstruction rbi = sm.process(rbs);

		if (rbi.catapult_in) {
			catapult.setIn();
		} else if (rbi.catapult_out) {
			catapult.setOut();
		} else {
			catapult.snap();
		}

		if (rbi.catapult_latch) {
			catapult.latch();
		}
		if (rbi.catapult_release) {
			catapult.release();
		}

		if (rbi.fingers_open) {
			fingers.open();
		} else if (rbi.fingers_close) {
			fingers.close();
		}

			if (input.getA())
				arm.setLow();
			else if (input.getX())
				arm.setMiddle();
			else if (input.getY())
				arm.setHigh();
			else
				arm.snap();
		
		if (arm.isHigh() && input.getLT()) {
			intake.setUp();
		} else if (input.getLB()) {
			intake.setDown();
		}
		
		if (input.getRB()) intake.setIn(); 
		else intake.setStop();
	}
	
	public void disabledPeriodic() {
		SmartDashboard.putNumber("Arm Disabled Pos", arm.getPos());
		SmartDashboard.putNumber("Catapult Disabled Pos", catapult.getPos());
	}
}
