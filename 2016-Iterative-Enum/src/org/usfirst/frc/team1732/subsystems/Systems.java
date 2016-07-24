package org.usfirst.frc.team1732.subsystems;

//import java.util.ArrayList;

import org.usfirst.frc.team1732.io.Input;
import org.usfirst.frc.team1732.statemachine.RobotInstruction;
import org.usfirst.frc.team1732.statemachine.RobotState;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Systems {
	private Drive drive = new Drive();
	private Intake intake = new Intake();
	private Arm arm = new Arm();
	private Catapult catapult = new Catapult();
	private Fingers fingers = new Fingers();
	private DefenseManipulator defense_manipulator = new DefenseManipulator();
	// private Camera camera = new Camera();
	private AnalogInput pressure = new AnalogInput(3); // on practice bot 3 is used for arm pot
	private Gyro gyro = new AnalogGyro(1);
	public RobotState robotState = new RobotState();
	public RobotInstruction robotInstruction = new RobotInstruction();

	boolean test_mode_started = false;

	private void resetDefense() {
		defense_manipulator.reset();
	}

	private void resetGyro() {
		gyro.reset();
	}

	private void resetDriveEncoders() {
		drive.reset();
	}

	public void getCameraState() {
		robotState.angle_to_goal = 0.5; // camera.getAngle();
		robotState.camera_exists = false; // camera.camera_exists;
	}

	public void getAutoState() {
		getState(false, false);
	}

	public void getState(boolean shoot, boolean reset_catapult) {
		// reset the state and instructions
		robotState = new RobotState();
		robotInstruction = new RobotInstruction();

		robotState.shoot = shoot;
		robotState.reset_catapult = reset_catapult;

		robotState.camera_exists = false; // camera.camera_exists;
		robotState.distance_to_goal = 0; // camera.getDistance();

		robotState.manip_encoder = defense_manipulator.getValue();
		SmartDashboard.putNumber("Manip_encoder", defense_manipulator.getValue());

		SmartDashboard.putNumber("Pressure", pressure.getValue() / 24.0);

		robotState.arm_aligned_high = arm.inDeadbandHigh();
		robotState.arm_aligned_middle = arm.inDeadbandMiddle();
		robotState.arm_aligned_low = arm.inDeadbandLow();
		robotState.arm_aligned_auto = arm.inDeadbandAuto();
		robotState.arm_aligned_cheval = arm.inDeadbandCheval();

		SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
		robotState.gyro = gyro.getAngle();

		robotState.shoot_mode_far = catapult.isFar();
		robotState.shoot_mode_close = catapult.isClose();
		robotState.shoot_mode_auto = catapult.isAuto();

		robotState.catapult_aligned_out = catapult.inDeadbandOut();
		robotState.catapult_aligned_load = catapult.inDeadbandLoad();
		robotState.catapult_aligned_shoot = catapult.inDeadbandShoot();

		robotState.intake_down = intake.isDown();
		robotState.intake_up = intake.isUp();

		robotState.drive_left_dist = drive.getLeft();
		robotState.drive_right_dist = drive.getRight();

		robotState.fingers_open = fingers.isOpen();
		robotState.fingers_closed = fingers.isClosed();
	}

	public void run() {
		drive.drive(-1 * robotInstruction.drive_left, -1 * robotInstruction.drive_right);

		if (robotInstruction.reset_defense) {
			defense_manipulator.reset();
		}
		if (robotInstruction.arm_auto) {
			arm.setAuto();
		}
		if (robotInstruction.reset_drive) {
			drive.reset();
		}

		if (robotInstruction.reset_gyro) {
			resetGyro();
		}

		if (robotInstruction.catapult_shoot) {
			catapult.setAuto(robotInstruction.catapult_auto_pos);
			catapult.setShoot();
		} else if (robotInstruction.catapult_out) {
			catapult.setOut();
		} else if (robotInstruction.catapult_load) {
			catapult.setLoad();
		} else {
			catapult.run();
		}

		if (robotInstruction.catapult_latch) {
			catapult.latch();
		} else if (robotInstruction.catapult_release) {
			catapult.release();
		}

		if (robotInstruction.arm_high) {
			arm.setHigh();
		} else if (robotInstruction.arm_middle) {
			arm.setMiddle();
		} else if (robotInstruction.arm_low) {
			arm.setLow();
		} else if (robotInstruction.arm_cheval) {
			arm.setCheval();
		} else {
			arm.run();
		}

		if (robotInstruction.fingers_open) {
			fingers.open();
		} else if (robotInstruction.fingers_close) {
			fingers.close();
		}

		if (robotInstruction.intake_up) {
			intake.setUp();
		} else if (robotInstruction.intake_down) {
			intake.setDown();
		}

		if (robotInstruction.intake_in) {
			intake.setIn();
		} else if (robotInstruction.intake_out) {
			intake.setOut();
		} else {
			intake.setStop();
		}

		if (robotInstruction.defense_up) {
			defense_manipulator.down();
		} else if (robotInstruction.defense_down) {
			defense_manipulator.up();
		} else {
			defense_manipulator.stop();
		}

	}

	public void run(Input io) {

		if (!robotInstruction.drive_auto)
			drive.drive(io.getLeftVert(), io.getRightVert());
		else
			drive.drive(robotInstruction.drive_left, robotInstruction.drive_right);
		if (io.getSetShootClose()) {
			catapult.setClose();
		} else if (io.getSetShootFar()) {
			catapult.setFar();
		} else if (io.getSetShootAuto()) {
			catapult.setAuto(robotInstruction.catapult_auto_pos);
		}

		if (robotInstruction.catapult_shoot) {
			catapult.setShoot();
		} else if (robotInstruction.catapult_out) {
			catapult.setOut();
		} else if (robotInstruction.catapult_load) {
			catapult.setLoad();
		} else {
			catapult.run();
		}

		if (robotInstruction.catapult_latch) {
			catapult.latch();
		} else if (robotInstruction.catapult_release) {
			catapult.release();
		}

		// if (io.getArmNot()) {
		if (io.getArmCheval() && fingers.isClosed()) {
			arm.setCheval();
		} else if (io.getArmLow() && fingers.isClosed()) {
			arm.setLow();// low
			/*
			 * if (arm.isLow()) { arm.run(); } else { if (catapult.inDeadbandLoad() /*&& intake.isDown()
			 *//*
				 * ) { robotInstruction.fingers_close = true; arm.setLow(); } else { arm.run(); } }
				 */
		} else if (io.getArmMiddle() && fingers.isClosed()) {
			arm.setMiddle(); // middle
			/*
			 * if (arm.isLow()) { if (catapult.inDeadbandLoad() /*&& intake.isDown()
			 *//*
				 * ) { robotInstruction.fingers_close = true; arm.setMiddle(); } else { arm.run(); } } else if (arm.isMiddle()) { arm.run(); } else {
				 * 
				 * arm.setMiddle(); // }
				 */
		} else if (io.getArmHigh() && fingers.isClosed()) {
			arm.setHigh(); // high
			/*
			 * if (arm.isLow()) { if (catapult.inDeadbandLoad() /*&& intake.isDown()*) { robotInstruction.fingers_close = true; arm.setHigh(); } else
			 * { arm.run(); } } else { arm.setHigh(); }
			 */
		} else {
			arm.run();
		} /*
			 * else { // run } if (arm.isAuto()) { arm.run(); } else { arm.stop(); } } /*} else if (io.getArmUp()) { arm.setUp(); } else if
			 * (io.getArmDown()) { arm.setDown();
			 * 
			 * } else { System.err.println("Unknown Arm State"); }
			 */
		boolean driverIntake = false;

		if (io.getFingersOpen()) {
			fingers.open();
			if (io.getTriggers() && arm.inDeadbandLow() && intake.isDown()) {
				intake.setIn();
				driverIntake = true;
			}
		} else if (io.getFingersClose()) {
			fingers.close();
		}

		// if (io.getIntakeNot()) {
		// if (robotInstruction.intake_up && (arm.inDeadbandHigh() ||
		// arm.inDeadbandMiddle())) {
		// intake.setUp();
		// } else if (robotInstruction.intake_down) {
		// intake.setDown();
		// }
		// } else { // default case
		if (io.getIntakeUp()) {
			intake.setUp();
		} else if (io.getIntakeDown()) {
			intake.setDown();
		}
		// }

		if (io.getIntakeIn()) { // TODO make intake auto for
								// raise and lower arm
			intake.setIn();
		} else if (io.getIntakeOut() && !driverIntake) {
			intake.setOut();
		} else if (!driverIntake) {
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

	public void test_mode(Input io) {
		/*
		 * In testing mode: wheel motors controlled by y-axis on sticks catapult latch is controlled by left stick button six capapult actuator is
		 * controlled by left stick buttons four (positive) and three (negative)
		 */

		if (!test_mode_started) {
			System.out.println("Test Mode Started");
			test_mode_started = true;
		}
		drive.drive(io.getLeftVert(), io.getRightVert());

		if (io.getRightSix())
			catapult.latch();
		else
			catapult.release();

		if (io.getLeftFour())
			catapult.testCatapultActuator(0.2);
		else if (io.getLeftThree())
			catapult.testCatapultActuator(-0.2);
		else
			catapult.testCatapultActuator(0);

		if (io.getRightFour())
			arm.testArmActuator(0.2);
		else if (io.getRightThree())
			arm.testArmActuator(-0.2);
		else
			arm.testArmActuator(0);

		if (io.getFingersOpen())
			fingers.open();
		else if (io.getFingersClose())
			fingers.close();

		if (io.getIntakeUp())
			intake.setUp();
		else if (io.getIntakeDown())
			intake.setDown();

		// TODO make intake auto for raise and lower arm
		if (io.getIntakeIn())
			intake.setIn();
		else if (io.getIntakeOut())
			intake.setOut();
		else
			intake.setStop();

		if (io.getManipulatorDown())
			defense_manipulator.down();
		else if (io.getManipulatorUp())
			defense_manipulator.up();
		else
			defense_manipulator.stop();

	}

	public void disabled() {
		SmartDashboard.putNumber("Arm Disabled Pos", arm.getPos());
		SmartDashboard.putNumber("Catapult Disabled Pos", catapult.getPos());
		SmartDashboard.putNumber("Encoder Left Disabled", drive.getLeft());
		SmartDashboard.putNumber("Encoder Right Disabled", drive.getRight());
		SmartDashboard.putNumber("Gyro Angle Disabled", gyro.getAngle());
		SmartDashboard.putNumber("Pressure", pressure.getValue() / 12.0);
		// camera.getAngle();
	}

	/*
	 * public void stopCamera() { camera.stopCamera(); }
	 * 
	 * public void startCamera() { //camera.startCamera(); }
	 */

	public void prepareAuto() {
		resetDriveEncoders();
		resetDefense();
		resetGyro();
		// catapult.setAuto(270);
		catapult.setClose();
		// camera.openCamera();
		// camera.startCapture();
	}
}
