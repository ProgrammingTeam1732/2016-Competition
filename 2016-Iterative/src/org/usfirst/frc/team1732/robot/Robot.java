
package org.usfirst.frc.team1732.robot;

import org.usfirst.frc.team1732.io.Input;
import org.usfirst.frc.team1732.statemachine.RobotInstruction;
import org.usfirst.frc.team1732.statemachine.RobotState;
import org.usfirst.frc.team1732.statemachine.State;
import org.usfirst.frc.team1732.statemachine.StateMachine;
import org.usfirst.frc.team1732.subsystems.Arm;
import org.usfirst.frc.team1732.subsystems.Catapult;
import org.usfirst.frc.team1732.subsystems.DefenseManipulator;
import org.usfirst.frc.team1732.subsystems.Drive;
import org.usfirst.frc.team1732.subsystems.Fingers;
import org.usfirst.frc.team1732.subsystems.Intake;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	Drive drive = new Drive();
	Intake intake = new Intake();
	Arm arm = new Arm();
	Catapult catapult = new Catapult();
	Fingers fingers = new Fingers();
	Input input = new Input();
	DefenseManipulator defense_manipulator = new DefenseManipulator();

	Encoder test = new Encoder(8, 9);
	
	StateMachine sm = new StateMachine();

	public void robotInit() {
		
		
		sm.addState(
			new State("Wait to Shoot",
				(RobotState rbs) -> {
					return new RobotInstruction();
				}, 
				(RobotState rbs) -> {
					if (rbs.shoot && rbs.fingers_open) return "Shoot Position";
					else if (rbs.shoot) return "Open Fingers";
					else return null;
				}
			)
		);
		
		sm.addState(
			new State("Shoot Position",
				(RobotState) -> {
					RobotInstruction rbi = new RobotInstruction();
					rbi.catapult_in = true;
					return rbi;
				},
				(RobotState rbs) -> {
					if (rbs.catapult_aligned_in && ((Math.abs(System.currentTimeMillis() - rbs.start_time) > 200))) return "Shoot";
					else return null;
				}
			)
		);
		
		sm.addState(
			new State("Open Fingers",
				(RobotState rbs) -> {
					RobotInstruction rbi = new RobotInstruction();
					rbi.fingers_open = true;
					return rbi;
				}, 
				(RobotState rbs) -> {
					if ((Math.abs(System.currentTimeMillis() - rbs.start_time) > 200)) return "Shoot Position";
					else return null;
				}
			)
		);
		
		sm.addState(
			new State("Shoot",
				(RobotState rbs) -> {
					RobotInstruction rbi = new RobotInstruction();
					rbi.catapult_release = true;
					return rbi;
				},
				(RobotState rbs) -> {
					if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 200) return "Retrive Tram";
					else return null;
				}
			)
		);
		
		sm.addState(
			new State("Retrive Tram",
				(RobotState rbs) -> {
					RobotInstruction rbi = new RobotInstruction();
					rbi.catapult_out = true;
					return rbi;
				}, 
				(RobotState rbs) -> {
					if (rbs.catapult_aligned_out) return "Latch Tram";
					else return null;
				}
			)
		);
		
		sm.addState(
			new State("Latch Tram",
				(RobotState rbs) -> {
					RobotInstruction rbi = new RobotInstruction();
					rbi.catapult_latch = true;
					return rbi;
				}, 
				(RobotState rbs) -> {
					if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 500) return "Pull Tram";
					else return null;
				}
			)
		);
		
		sm.addState(
			new State("Pull Tram",
				(RobotState rbs) -> {
					RobotInstruction rbi = new RobotInstruction();
					rbi.catapult_load = true;
					return rbi;
				},
				(RobotState rbs) -> {
					if (rbs.catapult_aligned_load) return "Wait to Shoot";
					else return null;
				}
			)
		);
	}

	public void teleopPeriodic() {
		SmartDashboard.putNumber("Test", test.get());
		
		drive.drive(input.getLeftVertJ() + input.getLeftVertC()/3, input.getRightVertJ() + input.getRightVertC()/3);

		RobotState rbs = new RobotState();
		rbs.shoot = input.getB();
		rbs.ball = input.getB();

		rbs.arm_aligned_high = arm.inDeadbandHigh();
		rbs.arm_aligned_middle = arm.inDeadbandMiddle();
		rbs.arm_aligned_low = arm.inDeadbandLow();

		rbs.catapult_aligned_out = catapult.inDeadbandOut();
		rbs.catapult_aligned_in = catapult.inDeadbandIn();
		rbs.catapult_aligned_load = catapult.inDeadbandLoad();

		rbs.drive_left_dist = drive.getLeft();
		rbs.drive_right_dist = drive.getRight();

		rbs.fingers_open = fingers.isOpen();
		rbs.fingers_closed = fingers.isClosed();

		RobotInstruction rbi = sm.process(rbs);

		if (rbi.catapult_in) {
			catapult.setIn();
		} else if (rbi.catapult_out) {
			catapult.setOut();
		} else if (rbi.catapult_load) {
			catapult.setLoad();
		} else {
			catapult.run();
		}

		if (rbi.catapult_latch) {
			catapult.latch();
		} else if (rbi.catapult_release) {
			catapult.release();
		}

		if (input.getRS()||rbi.fingers_open) {
			fingers.open();
		} else if (input.getLS()) {
			fingers.close();
		}

		if (input.getA() /*&& intake.isDown() && catapult.inDeadbandIn()*/)
			arm.setLow();
		else if (input.getX() && intake.isDown())
			arm.setMiddle();
		else if (input.getY() && intake.isDown())
			arm.setHigh();
		else
			arm.run();

		if (/*arm.isHigh() &&*/ input.getLB()) {
			intake.setUp();
		} else if (input.getLT()) {
			intake.setDown();
		}

		if (input.getRB()) {
			intake.setIn();
		} else if (input.getRT()) {
			intake.setOut();
		} else {
			intake.setStop();
		}

		if (input.getSELECT()) {
			defense_manipulator.down();
		} else if (input.getSTART()) {
			defense_manipulator.up();
		} else {
			defense_manipulator.stop();
		}
	}

	public void disabledPeriodic() {
		SmartDashboard.putNumber("Arm Disabled Pos", arm.getPos());
		SmartDashboard.putNumber("Catapult Disabled Pos", catapult.getPos());
		SmartDashboard.putNumber("Drive Left Dist", drive.getLeft());
		SmartDashboard.putNumber("Drive Right Dist", drive.getRight());
	}
}

// final String defaultAuto = "Default"; //final String customAuto = "Miato";
// //String autoSelected; //SendableChooser chooser;
// chooser = new SendableChooser(); //chooser.addDefault("Default Auto",
// defaultAuto); //chooser.addObject("My Auto", customAuto);
// //SmartDashboard.putData("Auto choices", chooser);
// public void autonomousInit() {//autoSelected = (String)
// chooser.getSelected(); //System.out.println("Auto selected: " +
// autoSelected); }
// public void autonomousPeriodic() { //switch (autoSelected) { //case
// customAuto: // Put custom auto code here // break; //case defaultAuto:
// //default: // Put default auto code here // break; //} //}
