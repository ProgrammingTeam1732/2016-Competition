
package org.usfirst.frc.team1732.robot;

import org.usfirst.frc.team1732.io.Input;
import org.usfirst.frc.team1732.statemachine.RobotInstruction;
import org.usfirst.frc.team1732.statemachine.RobotState;
import org.usfirst.frc.team1732.statemachine.State;
import org.usfirst.frc.team1732.statemachine.StateMachine;
import org.usfirst.frc.team1732.subsystems.Systems;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	Systems bot = new Systems();
	Input input = new Input();

	StateMachine rough_sm = new StateMachine();
	StateMachine rock_wall_sm = new StateMachine();
	StateMachine ramparts_sm = new StateMachine();
	StateMachine moat_sm = new StateMachine();
	StateMachine portcullis_sm = new StateMachine();
	StateMachine cheval_sm = new StateMachine();
	StateMachine drawbridge_sm = new StateMachine();
	StateMachine sally_port_sm = new StateMachine();
	StateMachine low_bar_sm = new StateMachine();

	StateMachine sm = new StateMachine();

	public void robotInit() {

		chooser.addDefault(default_auto, default_auto);
		chooser.addObject(rough, rough);
		chooser.addObject(ramparts, ramparts);
		chooser.addObject(moat, moat);
		chooser.addObject(portcullis, portcullis);
		chooser.addObject(low_bar, low_bar);
		chooser.addObject(cheval, cheval);
		chooser.addObject(sally_port, sally_port);
		chooser.addObject(drawbridge, drawbridge);
		chooser.addObject(rock_wall, rock_wall);
		SmartDashboard.putData("Auto Mode", chooser);

		after_chooser.addDefault(default_auto, default_auto);
		after_chooser.addObject(shoot, shoot);
		SmartDashboard.putData("After Mode", after_chooser);

		sm.addState(new State("Wait to Shoot", (RobotState rbs) -> {
			return new RobotInstruction();
		} , (RobotState rbs) -> {
			if (rbs.shoot && rbs.fingers_open)
				return "Shoot Position";
			else if (rbs.shoot)
				return "Open Fingers";
			else
				return null;
		}));

		sm.addState(new State("Shoot Position", (RobotState) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.catapult_in = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (rbs.catapult_aligned_in && ((Math.abs(System.currentTimeMillis() - rbs.start_time) > 500)))
				return "Shoot";
			else
				return null;
		}));

		sm.addState(new State("Open Fingers", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.fingers_open = true;
			return rbi;
		} , (RobotState rbs) -> {
			if ((Math.abs(System.currentTimeMillis() - rbs.start_time) > 200))
				return "Shoot Position";
			else
				return null;
		}));

		sm.addState(new State("Shoot", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.catapult_release = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 200)
				return "Retrive Tram";
			else
				return null;
		}));

		sm.addState(new State("Retrive Tram", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.catapult_out = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (rbs.catapult_aligned_out)
				return "Latch Tram";
			else
				return null;
		}));

		sm.addState(new State("Latch Tram", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.catapult_latch = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 500)
				return "Pull Tram";
			else
				return null;
		}));

		sm.addState(new State("Pull Tram", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.catapult_load = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (rbs.catapult_aligned_load)
				return "Wait to Shoot";
			else
				return null;
		}));

		rough_sm.setState("Arm Down");
		rough_sm.addState(new State("Arm Down", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.arm_middle = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (rbs.arm_aligned_middle)
				return "Drive Acc";
			else
				return null;
		})).addState(new State("Drive Acc", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.drive_left = 0.8 * ((System.currentTimeMillis() - rbs.start_time) / 1000.0);
			rbi.drive_right = 0.8 * ((System.currentTimeMillis() - rbs.start_time) / 1000.0);
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 1000)
				return "Drive Steady";
			else
				return null;
		})).addState(new State("Drive Steady", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.drive_left = 0.8;
			rbi.drive_right = 0.8;
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 1000)
				return "Drive Slow";
			else
				return null;
		})).addState(new State("Drive Slow", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.drive_left = 0.3;
			rbi.drive_right = 0.3;
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 3000)
				return "Arm Up";
			else
				return null;
		})).addState(new State("Arm Up", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.arm_high = true;
			rbi.drive_left = 0.1;
			rbi.drive_right = 0.1;
			return rbi;
		} , (RobotState rbs) -> {
			if (rbs.arm_aligned_high)
				return "Shoot Position";
			return null;
		})).addState(new State("Shoot Position", (RobotState) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.fingers_open = true;
			rbi.catapult_in = true;
			rbi.drive_left = 0.2;
			rbi.drive_right = 0.2;
			return rbi;
		} , (RobotState rbs) -> {
			if (rbs.catapult_aligned_in && ((Math.abs(System.currentTimeMillis() - rbs.start_time) > 500)))
				return "Shoot";
			else
				return null;
		})).addState(new State("Shoot", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.catapult_release = true;
			rbi.drive_left = 0.2;
			rbi.drive_right = 0.2;
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 200)
				return "Retrive Tram";
			else
				return null;
		})).addState(new State("Retrive Tram", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.catapult_out = true;
			rbi.drive_left = -0.2;
			rbi.drive_right = -0.2;
			return rbi;
		} , (RobotState rbs) -> {
			if (rbs.catapult_aligned_out)
				return "Latch Tram";
			else
				return null;
		})).addState(new State("Latch Tram", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.catapult_latch = true;
			rbi.drive_left = -0.2;
			rbi.drive_right = -0.2;
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 500)
				return "Pull Tram";
			else
				return null;
		})).addState(new State("Pull Tram", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.catapult_load = true;
			rbi.drive_left = -0.2;
			rbi.drive_right = -0.2;
			return rbi;
		} , (RobotState rbs) -> {
			if (rbs.catapult_aligned_load)
				return "Stop";
			else
				return null;
		})).addState(new State("Stop", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			return rbi;
		} , (RobotState rbs) -> {
			return null;
		}));
	}

	public void teleopPeriodic() {
		bot.run(sm.process(bot.getState(input.getB())), input);
	}

	public void disabledPeriodic() {
		bot.disabled();
	}

	long start_time;

	boolean once = true;

	final String default_auto = "Default";
	final String rough = "Rough";
	final String rock_wall = "Wall";
	final String ramparts = "Ramparts";
	final String moat = "Moat";
	final String portcullis = "Portcullis";
	final String cheval = "Cheval";
	final String drawbridge = "Draw Bridge";
	final String sally_port = "Sally Port";
	final String low_bar = "Low Bar";
	String auto_mode;
	SendableChooser chooser = new SendableChooser();

	final String shoot = "Shoot";
	SendableChooser after_chooser = new SendableChooser();

	public void autonomousInit() {
		auto_mode = (String) chooser.getSelected();
		rough_sm.setState("Arm Down");
	}

	public void autonomousPeriodic() {
		if (auto_mode.equals(default_auto)) {
			bot.disabled();
		} else if (auto_mode.equals(rough)) {
			bot.run(rough_sm.process(bot.getState()));
		} else if (auto_mode.equals(rock_wall)) {
			// if (System.currentTimeMillis() - start_time < 1750) {
			// drive.drive(-0.8, -0.8);
			// }
			// else {
			// drive.drive(0, 0);
			// }
		} else if (auto_mode.equals(moat)) {
			// if (System.currentTimeMillis() - start_time < 700) {
			// intake.setDown();
			// arm.setMiddle();
			// } else if (System.currentTimeMillis() - start_time < 2800) {
			// drive.drive(-0.8, -0.8);
			// arm.setMiddle();
			// }
			// else {
			// drive.drive(0, 0);
			// arm.setMiddle();
			// }
		} else if (auto_mode.equals(cheval)) {
			// TODO
			// drive.drive(0, 0);
		} else if (auto_mode.equals(low_bar)) {
			// if (!arm.inDeadbandLow()) {
			// intake.setDown();
			// arm.setLow();
			// }
			// else if (once) {
			// start_time = System.currentTimeMillis();
			// once = false;
			// }
			// if (arm.inDeadbandLow() && System.currentTimeMillis() -
			// start_time < 3000) {
			// drive.drive(-0.4, -0.4);
			// arm.setLow();
			// }
			// else {
			// drive.drive(0, 0);
			// arm.setLow();
			// }
		} else if (auto_mode.equals(drawbridge)) {
			// TODO
			// drive.drive(0, 0);
		} else if (auto_mode.equals(portcullis)) {
			// TODO
			// drive.drive(0, 0);
		} else if (auto_mode.equals(ramparts)) {
			// if (System.currentTimeMillis() - start_time < 2000) {
			// drive.drive(-0.5, -0.5);
			// }
			// else {
			// drive.drive(0, 0);
			// }
		} else if (auto_mode.equals(sally_port)) {
			// TODO
			// drive.drive(0, 0);
		} else {
			// drive.drive(0, 0);
			// System.err.println("Oops");
		}
	}

}