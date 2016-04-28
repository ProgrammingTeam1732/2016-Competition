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
	Drive drive = new Drive();
	Intake intake = new Intake();
	Arm arm = new Arm();
	Catapult catapult = new Catapult();
	Fingers fingers = new Fingers();
	DefenseManipulator defense_manipulator = new DefenseManipulator();
	Camera camera = new Camera();
	AnalogInput pressure = new AnalogInput(3); //on practice bot 3 is used for arm pot

	boolean test_mode_started = false;

	//AnalogInput pressure = new AnalogInput(3);

	Gyro gyro = new AnalogGyro(1);
	
	public void resetDefense() {
		defense_manipulator.reset();
	}

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
		
		rbs.angle_to_goal = camera.getAngle();
		rbs.camera_exists = camera.camera_exists;
		
		rbs.manip_encoder = defense_manipulator.getValue();
		SmartDashboard.putNumber("Manip_encoder", defense_manipulator.getValue());
		
		SmartDashboard.putNumber("Pressure", pressure.getValue() / 24.0);

		rbs.arm_aligned_high = arm.inDeadbandHigh();
		rbs.arm_aligned_middle = arm.inDeadbandMiddle();
		rbs.arm_aligned_low = arm.inDeadbandLow();
		rbs.arm_aligned_auto = arm.inDeadbandAuto();

		SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
		rbs.gyro = gyro.getAngle();

		rbs.shoot_mode_far = catapult.isFar();
		rbs.shoot_mode_close = catapult.isClose();
		rbs.shoot_mode_auto = catapult.isAuto();
		
		rbs.catapult_aligned_out = catapult.inDeadbandOut();
		rbs.catapult_aligned_load = catapult.inDeadbandLoad();
		rbs.catapult_aligned_shoot = catapult.inDeadbandShoot();

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
		
		rbs.camera_exists = camera.camera_exists;
		rbs.distance_to_goal = camera.getDistance();
		
		rbs.manip_encoder = defense_manipulator.getValue();
		SmartDashboard.putNumber("Manip_encoder", defense_manipulator.getValue());
		
		SmartDashboard.putNumber("Pressure", pressure.getValue() / 24.0);

		rbs.arm_aligned_high = arm.inDeadbandHigh();
		rbs.arm_aligned_middle = arm.inDeadbandMiddle();
		rbs.arm_aligned_low = arm.inDeadbandLow();
		rbs.arm_aligned_auto = arm.inDeadbandAuto();

		SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
		rbs.gyro = gyro.getAngle();

		rbs.shoot_mode_far = catapult.isFar();
		rbs.shoot_mode_close = catapult.isClose();
		rbs.shoot_mode_auto = catapult.isAuto();
		
		rbs.catapult_aligned_out = catapult.inDeadbandOut();
		rbs.catapult_aligned_load = catapult.inDeadbandLoad();
		rbs.catapult_aligned_shoot = catapult.inDeadbandShoot();

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

		if (rbi.reset_defense) {
			defense_manipulator.reset();
		}
		if (rbi.arm_auto) {
			arm.setAuto();
		}
		if (rbi.reset_drive) {
			drive.reset();
		}

		if (rbi.reset_gyro) {
			resetGyro();
		}

		if (rbi.catapult_shoot) {
			catapult.setAuto(rbi.catapult_auto_pos);
			catapult.setShoot();
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
		
		if(!rbi.drive_auto) drive.drive(io.getLeftVert(), io.getRightVert());
		else drive.drive(rbi.drive_left, rbi.drive_right);
		if (io.getSetShootClose()) {
			catapult.setClose();
		} else if (io.getSetShootFar()) {
			catapult.setFar();
		} else if (io.getSetShootAuto()) {
			catapult.setAuto(rbi.catapult_auto_pos);
		}
		
		if (rbi.catapult_shoot) {
			catapult.setShoot();
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

		if (io.getFingersOpen()) {
			fingers.open();
		} else if (io.getFingersClose()) {
			fingers.close();
		}

		// if (io.getIntakeNot()) {
		// if (rbi.intake_up && (arm.inDeadbandHigh() ||
		// arm.inDeadbandMiddle())) {
		// intake.setUp();
		// } else if (rbi.intake_down) {
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
		} else if (io.getIntakeOut()) {
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

	public void test_mode(Input io) {
		/*
		 * In testing mode: wheel motors controlled by y-axis on sticks catapult
		 * latch is controlled by left stick button six capapult actuator is
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
		//SmartDashboard.putNumber("Pressure", pressure.getValue() / 24.0);
		//camera.getAngle();
	}

	/*public void stopCamera() {
		camera.stopCamera();
	}
	
	public void startCamera() {
		//camera.startCamera();
	}*/
	
	public void prepareAuto() {
		drive.reset();
		defense_manipulator.reset();
		catapult.setAuto(270);
		//catapult.setClose();
		//camera.openCamera();
		//camera.startCapture();
	}
}
