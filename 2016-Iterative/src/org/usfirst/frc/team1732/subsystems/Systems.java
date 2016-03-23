package org.usfirst.frc.team1732.subsystems;

import java.util.ArrayList;

import org.usfirst.frc.team1732.io.Input;
import org.usfirst.frc.team1732.statemachine.RobotInstruction;
import org.usfirst.frc.team1732.statemachine.RobotState;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Systems {
	Drive drive = new Drive();
	Intake intake = new Intake();
	Arm arm = new Arm();
	Catapult catapult = new Catapult();
	Fingers fingers = new Fingers();
	DefenseManipulator defense_manipulator = new DefenseManipulator();
	Camera camera = new Camera();

	Gyro gyro = new AnalogGyro(1);

	public void resetGyro() {
		gyro.reset();
	}

	public void resetDriveEncoders() {
		drive.reset();
	}

	public RobotState getState() {
		return getState(false);
	}

	public RobotState getCameraState() {
		RobotState rbs = new RobotState();

		rbs.shoot = false;

		SmartDashboard.putNumber("Camera Angle", camera.getAngle());
		
		rbs.camera_angle = camera.getAngle();

		rbs.arm_aligned_high = arm.inDeadbandHigh();
		rbs.arm_aligned_middle = arm.inDeadbandMiddle();
		rbs.arm_aligned_low = arm.inDeadbandLow();

		SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
		rbs.gyro = gyro.getAngle();

		rbs.catapult_aligned_out = catapult.inDeadbandOut();
		rbs.catapult_aligned_in = catapult.inDeadbandIn();
		rbs.catapult_aligned_load = catapult.inDeadbandLoad();

		rbs.intake_down = intake.isDown();
		rbs.intake_up = intake.isUp();

		rbs.drive_left_dist = drive.getLeft();
		rbs.drive_right_dist = drive.getRight();

		rbs.fingers_open = fingers.isOpen();
		rbs.fingers_closed = fingers.isClosed();
		return rbs;
	}

	public RobotState getState(boolean shoot) {
		RobotState rbs = new RobotState();

		rbs.shoot = shoot;

		rbs.arm_aligned_high = arm.inDeadbandHigh();
		rbs.arm_aligned_middle = arm.inDeadbandMiddle();
		rbs.arm_aligned_low = arm.inDeadbandLow();

		SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
		rbs.gyro = gyro.getAngle();

		rbs.catapult_aligned_out = catapult.inDeadbandOut();
		rbs.catapult_aligned_in = catapult.inDeadbandIn();
		rbs.catapult_aligned_load = catapult.inDeadbandLoad();

		rbs.intake_down = intake.isDown();
		rbs.intake_up = intake.isUp();

		rbs.drive_left_dist = drive.getLeft();
		rbs.drive_right_dist = drive.getRight();

		rbs.fingers_open = fingers.isOpen();
		rbs.fingers_closed = fingers.isClosed();
		return rbs;
	}

	public void run(RobotInstruction rbi) {
		drive.drive(-1 * rbi.drive_left, -1 * rbi.drive_right);

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

		if (rbi.arm_high) {
			arm.setHigh();
		} else if (rbi.arm_middle) {
			arm.setMiddle();
		} else if (rbi.arm_low) {
			arm.setLow();
		} else {
			arm.run();
		}

		if (rbi.fingers_open) {
			fingers.open();
		} else if (rbi.fingers_close) {
			fingers.close();
		}

		if (rbi.intake_up) {
			intake.setUp();
		} else if (rbi.intake_down) {
			intake.setDown();
		}

		if (rbi.intake_in) {
			intake.setIn();
		} else if (rbi.intake_out) {
			intake.setOut();
		} else {
			intake.setStop();
		}

		if (rbi.defense_up) {
			defense_manipulator.down();
		} else if (rbi.defense_down) {
			defense_manipulator.up();
		} else {
			defense_manipulator.stop();
		}
	}

	public void run(RobotInstruction rbi, Input io) {
		drive.drive(io.getLeftVert(), io.getRightVert());

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

		// if (io.getArmNot()) {
		if (io.getArmLow() && fingers.isClosed()) {
			arm.setLow();// low
			/*
			 * if (arm.isLow()) { arm.run(); } else { if
			 * (catapult.inDeadbandLoad() /*&& intake.isDown()
			 *//*
				 * ) { rbi.fingers_close = true; arm.setLow(); } else {
				 * arm.run(); } }
				 */
		} else if (io.getArmMiddle() && fingers.isClosed()) {
			arm.setMiddle(); // middle
			/*
			 * if (arm.isLow()) { if (catapult.inDeadbandLoad() /*&&
			 * intake.isDown()
			 *//*
				 * ) { rbi.fingers_close = true; arm.setMiddle(); } else {
				 * arm.run(); } } else if (arm.isMiddle()) { arm.run(); } else {
				 * 
				 * arm.setMiddle(); // }
				 */
		} else if (io.getArmHigh() && fingers.isClosed()) {
			arm.setHigh(); // high
			/*
			 * if (arm.isLow()) { if (catapult.inDeadbandLoad() /*&&
			 * intake.isDown()*) { rbi.fingers_close = true; arm.setHigh(); }
			 * else { arm.run(); } } else { arm.setHigh(); }
			 */
		} else {
			arm.run();
		} /*
			 * else { // run } if (arm.isAuto()) { arm.run(); } else {
			 * arm.stop(); } } /*} else if (io.getArmUp()) { arm.setUp(); } else
			 * if (io.getArmDown()) { arm.setDown();
			 * 
			 * } else { System.err.println("Unknown Arm State"); }
			 */

		if (io.getFingersNot()) {
			if (rbi.fingers_open) {
				fingers.open();
			} else if (rbi.fingers_close) {
				fingers.close();
			}
		} else {
			if (io.getFingersOpen()) {
				fingers.open();
			} else if (io.getFingersClose()) {
				fingers.close();
			}
		}

		// if (io.getIntakeNot()) {
		// if (rbi.intake_up && (arm.inDeadbandHigh() ||
		// arm.inDeadbandMiddle())) {
		// intake.setUp();
		// } else if (rbi.intake_down) {
		// intake.setDown();
		// }
		// } else { // default case
		if (io.getClimberUp()) {
			intake.setUp();
		} else if (io.getClimberDown()) {
			intake.setDown();
		}
		// }

		if (io.getIntakeIn() || rbi.intake_in) { // TODO make intake auto for
													// raise and lower arm
			intake.setIn();
		} else if (io.getIntakeOut() || rbi.intake_out) {
			intake.setOut();
		} else {
			intake.setStop();
		}

		if (io.getManipulatorDown()) {
			defense_manipulator.down();
		} else if (io.getManipulatorUp()) {
			defense_manipulator.up();
		} else {
			defense_manipulator.stop();
		}
	}

	public void disabled() {
		SmartDashboard.putNumber("Arm Disabled Pos", arm.getPos());
		SmartDashboard.putNumber("Catapult Disabled Pos", catapult.getPos());
		SmartDashboard.putNumber("Drive Left Dist", drive.getLeft());
		SmartDashboard.putNumber("Drive Right Dist", drive.getRight());
		SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
	}

	public void startCamera() {
		camera.startCamera();
	}
}