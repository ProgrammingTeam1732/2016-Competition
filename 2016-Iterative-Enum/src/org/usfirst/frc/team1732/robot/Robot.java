
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

	public static Systems bot;
	public Input input = new Input();

	StateMachine<cross_terrain_states> cross_terrain_sm = new StateMachine<cross_terrain_states>();

	public enum cross_terrain_states {
		ArmDown, DriveAcc, DriveSteady, Finished;
	}

	StateMachine<portcullis_states> portcullis_sm = new StateMachine<portcullis_states>();

	public enum portcullis_states {
		Accelerate, ContinueArmDefense, ContinueDrive, ArmLow, Finished;
	}

	StateMachine<cheval_states> cheval_sm = new StateMachine<cheval_states>();

	public enum cheval_states {
		DriveToCheval, DropIntake, DropArmToCheval, DriveAcrossCheval, Finished;
	}

	StateMachine<drawbridge_states> drawbridge_sm = new StateMachine<drawbridge_states>();

	public enum drawbridge_states {
		Accelerate, Finished;
	}

	StateMachine<sally_port_states> sally_port_sm = new StateMachine<sally_port_states>();

	public enum sally_port_states {
		Accelerate, Finished;
	}

	StateMachine<low_bar_states> low_bar_sm = new StateMachine<low_bar_states>();

	public enum low_bar_states {
		DropIntake, DropArm, Wait, DriveForward, Turn, DriveForwardTwo, RaiseArm, OpenFingers, Finished;
	}

	StateMachine<position_two_states> position_two_sm = new StateMachine<position_two_states>();

	public enum position_two_states {
		Center, DriveForward, Turn, DriveForwardTwo, Finished;
	}

	StateMachine<position_three_states> position_three_sm = new StateMachine<position_three_states>();

	public enum position_three_states {
		TurnRight, DriveForward, TurnLeft, DriveForwardTwo, Finished
	}

	StateMachine<position_four_states> position_four_sm = new StateMachine<position_four_states>();

	public enum position_four_states {
		TurnLeft, DriveForward, TurnRight, DriveForwardTwo, Finished;
	}

	StateMachine<position_five_states> position_five_sm = new StateMachine<position_five_states>();

	public enum position_five_states {
		Center, DriveForward, Turn, DriveForwardTwo, Finished;
	}

	StateMachine<approach_states> approach_sm = new StateMachine<approach_states>();

	public enum approach_states {
		Accelerate, Finished;
	}

	StateMachine<shoot_states> shoot_sm = new StateMachine<shoot_states>();

	public enum shoot_states {
		WaitToShoot, AutoInit, ShootMode, PointAtGoal, AutoShootPosition, ShootPosition, Shoot, RetrieveTram, LatchTram, PullTram;
	}

	public void robotInit() {
		bot = new Systems();
		start_chooser.addDefault(do_nothing, do_nothing); // Stop
		start_chooser.addObject(cross_defenses, cross_defenses); // Cross defenses
		start_chooser.addObject(approach_defenses, approach_defenses); // Approach defenses
		SmartDashboard.putData("Start Mode", start_chooser);

		defense_chooser.addDefault(do_nothing, do_nothing);
		defense_chooser.addObject(cross_terrain, cross_terrain);
		defense_chooser.addObject(portcullis, portcullis);
		defense_chooser.addObject(low_bar, low_bar);
		defense_chooser.addObject(cheval, cheval);
		defense_chooser.addObject(sally_port, sally_port);
		defense_chooser.addObject(drawbridge, drawbridge);
		SmartDashboard.putData("Defense", defense_chooser);

		position_chooser.addDefault(position_one, position_one);
		position_chooser.addObject(position_two, position_two);
		position_chooser.addObject(position_three, position_three);
		position_chooser.addObject(position_four, position_four);
		position_chooser.addObject(position_five, position_five);
		position_chooser.addObject(do_nothing, do_nothing);
		SmartDashboard.putData("Position", position_chooser);

		shoot_sm.addState(shoot_states.WaitToShoot, () -> {
			if (bot.robotState.shoot && bot.robotState.fingers_open)
				return shoot_states.ShootMode;
			if (bot.robotState.reset_catapult && bot.robotState.fingers_open)
				return shoot_states.ShootMode;
			else
				return null;
		}).addState(shoot_states.AutoInit, () -> {
			if (bot.robotState.arm_aligned_high)
				return shoot_states.ShootMode;
			else
				return null;
		}).addState(shoot_states.ShootMode, () -> {
			if (bot.robotState.shoot_mode_far || bot.robotState.shoot_mode_close)
				return shoot_states.ShootPosition;
			else if (!bot.robotState.camera_exists)
				return shoot_states.WaitToShoot;
			else if (bot.robotState.shoot_mode_auto)
				return shoot_states.PointAtGoal;
			else
				return null;
		}).addState(shoot_states.PointAtGoal, () -> {
			if (bot.robotState.angle_to_goal == -1.0) {
				bot.robotInstruction.drive_left = 0.2;
				bot.robotInstruction.drive_right = -0.2;
			}
			double turn = -0.2;
			if (bot.robotState.angle_to_goal < 0.5)
				turn = 0.2;
			else
				turn = -0.2;
			bot.robotInstruction.drive_auto = true;
			bot.robotInstruction.drive_left = turn;
			bot.robotInstruction.drive_right = -turn;
		}, () -> {
			// FIXME: change this after testing turning
			if (Math.abs(0.5 - bot.robotState.angle_to_goal) < 0.1)
				return shoot_states.WaitToShoot;
			else if (!bot.robotState.shoot_mode_auto)
				return shoot_states.WaitToShoot;
			else
				return null;
		}).addState(shoot_states.AutoShootPosition, () -> {
			// TODO: calculate function for setpoint based on distance
			// bot.robotInstruction.catapult_auto_pos = (int) (bot.robotState.distance_to_goal * 2);
			bot.robotInstruction.catapult_shoot = true;
		}, () -> {
			if (bot.robotState.catapult_aligned_shoot && bot.robotState.arm_aligned_high && bot.robotState.fingers_open
					&& ((Math.abs(System.currentTimeMillis() - bot.robotState.start_time) > 500)))
				return shoot_states.Shoot;
			else
				return null;
		}).addState(shoot_states.ShootPosition, () -> {
			bot.robotInstruction.catapult_shoot = true;
		}, () -> {
			if (bot.robotState.catapult_aligned_shoot && bot.robotState.arm_aligned_high && bot.robotState.fingers_open
					&& ((Math.abs(System.currentTimeMillis() - bot.robotState.start_time) > 500)))
				return shoot_states.Shoot;
			if (bot.robotState.reset_catapult && bot.robotState.fingers_open)
				return shoot_states.Shoot;
			else
				return null;
		}).addState(shoot_states.Shoot, () -> {
			bot.robotInstruction.catapult_release = true;
		}, () -> {
			if (Math.abs(System.currentTimeMillis() - bot.robotState.start_time) > 200)
				return shoot_states.RetrieveTram;
			else
				return null;
		}).addState(shoot_states.RetrieveTram, () -> {
			bot.robotInstruction.catapult_out = true;
		}, () -> {
			if (bot.robotState.catapult_aligned_out)
				return shoot_states.LatchTram;
			else
				return null;
		}).addState(shoot_states.LatchTram, () -> {
			bot.robotInstruction.catapult_latch = true;
		}, () -> {
			if (Math.abs(System.currentTimeMillis() - bot.robotState.start_time) > 500)
				return shoot_states.PullTram;
			else
				return null;
		}).addState(shoot_states.PullTram, () -> {
			bot.robotInstruction.catapult_load = true;
		}, () -> {
			if (bot.robotState.catapult_aligned_load)
				return shoot_states.WaitToShoot;
			else
				return null;
		});

		cross_terrain_sm.addState(cross_terrain_states.ArmDown, () -> {
			bot.robotInstruction.intake_down = true;
			bot.robotInstruction.arm_middle = true;
		}, () -> { // FIXME?
			if (bot.robotState.arm_aligned_middle)
				return cross_terrain_states.DriveAcc;
			else
				return null;
		}).addState(cross_terrain_states.DriveAcc, () -> {
			bot.robotInstruction.drive_left = 0.8 * ((System.currentTimeMillis() - bot.robotState.start_time) / 2500.0);
			bot.robotInstruction.drive_right = 0.8 * ((System.currentTimeMillis() - bot.robotState.start_time) / 2500.0);
		}, () -> {
			if (Math.abs(System.currentTimeMillis() - bot.robotState.start_time) > 2500)
				return cross_terrain_states.DriveSteady;
			else
				return null;
		}).addState(cross_terrain_states.DriveSteady, () -> {
			bot.robotInstruction.drive_left = 0.8 - (0.8 * ((System.currentTimeMillis() - bot.robotState.start_time) / 3000.0));
			bot.robotInstruction.drive_right = 0.8 - (0.8 * ((System.currentTimeMillis() - bot.robotState.start_time) / 3000.0));
		}, () -> {
			if (Math.abs(System.currentTimeMillis() - bot.robotState.start_time) > 3000)
				return cross_terrain_states.Finished;
			else
				return null;
		}).addState(cross_terrain_states.Finished, () -> {
			return null;
		});

		approach_sm.addState(approach_states.Accelerate, () -> {
			bot.robotInstruction.drive_left = 0.5 * ((System.currentTimeMillis() - bot.robotState.start_time) / 1000.0);
			bot.robotInstruction.drive_right = 0.5 * ((System.currentTimeMillis() - bot.robotState.start_time) / 1000.0);
		}, () -> {
			if (Math.abs(System.currentTimeMillis() - bot.robotState.start_time) > 1000)
				return approach_states.Finished;
			else
				return null;
		}).addState(approach_states.Finished, () -> {
			return null;
		});

		low_bar_sm.addState(low_bar_states.DropIntake, () -> {
			bot.robotInstruction.intake_down = true;
		}, () -> {
			if (Math.abs(System.currentTimeMillis() - bot.robotState.start_time) > 1000)
				return low_bar_states.DropArm;
			else
				return null;
		}).addState(low_bar_states.DropArm, () -> {
			bot.robotInstruction.catapult_load = true;
			bot.robotInstruction.fingers_close = true;
			if (bot.robotState.catapult_aligned_load && bot.robotState.fingers_closed && bot.robotState.intake_down) {
				bot.robotInstruction.arm_low = true;
			}
		}, () -> {
			if (Math.abs(System.currentTimeMillis() - bot.robotState.start_time) > 1000)
				return low_bar_states.Wait;
			else
				return null;
		}).addState(low_bar_states.Wait, () -> {
			if (bot.robotState.arm_aligned_low)
				return low_bar_states.DriveForward;
			else
				return null;
		}).addState(low_bar_states.DriveForward, () -> {
			bot.robotInstruction.reset_gyro = true;
			bot.robotInstruction.drive_left = 0.345;
			bot.robotInstruction.drive_right = 0.345;
		}, () -> {
			if (((bot.robotState.drive_left_dist + bot.robotState.drive_left_dist) / 2.0) * 0.095 > 220) {
				return low_bar_states.Turn;
			}
			return null;
		}).addState(low_bar_states.Turn, () -> {
			if (bot.robotState.gyro > 55) {
				bot.robotInstruction.drive_left = -0.4;
				bot.robotInstruction.drive_right = 0.4;
			} else {
				bot.robotInstruction.drive_left = 0.4;
				bot.robotInstruction.drive_right = -0.4;
			}
			bot.robotInstruction.reset_drive = true;
		}, () -> {
			if (Math.abs(bot.robotState.gyro - 55) < 1) {
				return low_bar_states.DriveForwardTwo;
			}
			return null;
		}).addState(low_bar_states.DriveForwardTwo, () -> {
			bot.robotInstruction.drive_left = 0.2;
			bot.robotInstruction.drive_right = 0.2;
		}, () -> {
			if (((bot.robotState.drive_left_dist + bot.robotState.drive_left_dist) / 2.0) * 0.095 > 160) {
				return low_bar_states.RaiseArm;
			}
			return null;
		}).addState(low_bar_states.RaiseArm, () -> {
			bot.robotInstruction.arm_high = true;
		}, () -> {
			if (System.currentTimeMillis() - bot.robotState.start_time > 500)
				return low_bar_states.OpenFingers;
			else
				return null;
		}).addState(low_bar_states.OpenFingers, () -> {
			bot.robotInstruction.fingers_open = true;
		}, () -> {
			if (System.currentTimeMillis() - bot.robotState.start_time > 500)
				return low_bar_states.Finished;
			else
				return null;
		}).addState(low_bar_states.Finished, () -> {
			return null;
		});

		position_two_sm.addState(position_two_states.Center, () -> {
			if (bot.robotState.gyro > 0) {
				bot.robotInstruction.drive_left = -0.4;
				bot.robotInstruction.drive_right = 0.4;
			} else {
				bot.robotInstruction.drive_left = 0.4;
				bot.robotInstruction.drive_right = -0.4;
			}
			bot.robotInstruction.reset_drive = true;
		}, () -> {
			if (Math.abs(bot.robotState.gyro) < 1) {
				return position_two_states.DriveForward;
			}
			return null;
		}).addState(position_two_states.DriveForward, () -> {
			bot.robotInstruction.reset_gyro = true;
			bot.robotInstruction.drive_left = 0.345;
			bot.robotInstruction.drive_right = 0.345;
		}, () -> {
			if (((bot.robotState.drive_left_dist + bot.robotState.drive_left_dist) / 2.0) * 0.095 > 105) {
				return position_two_states.Turn;
			}
			return null;
		}).addState(position_two_states.Turn, () -> {
			if (bot.robotState.gyro > +55) {
				bot.robotInstruction.drive_left = -0.4;
				bot.robotInstruction.drive_right = 0.4;
			} else {
				bot.robotInstruction.drive_left = 0.4;
				bot.robotInstruction.drive_right = -0.4;
			}
			bot.robotInstruction.reset_drive = true;
		}, () -> {
			if (Math.abs(bot.robotState.gyro - 55) < 1) {
				return position_two_states.DriveForwardTwo;
			}
			return null;
		}).addState(position_two_states.DriveForwardTwo, () -> {
			bot.robotInstruction.drive_left = 0.2;
			bot.robotInstruction.drive_right = 0.2;
		}, () -> {
			if (((bot.robotState.drive_left_dist + bot.robotState.drive_left_dist) / 2.0) * 0.095 > 53) {
				return position_two_states.Finished;
			}
			return null;
		}).addState(position_two_states.Finished, () -> {
			bot.robotInstruction.arm_high = true;
			bot.robotInstruction.fingers_open = true;
		}, () -> {
			return null;
		});

		position_three_sm.addState(position_three_states.TurnRight, () -> {
			if (bot.robotState.gyro > 40) {
				bot.robotInstruction.drive_left = -0.4;
				bot.robotInstruction.drive_right = 0.4;
			} else {
				bot.robotInstruction.drive_left = 0.4;
				bot.robotInstruction.drive_right = -0.4;
			}
			bot.robotInstruction.reset_drive = true;
		}, () -> {
			if (Math.abs(bot.robotState.gyro - 40) < 1) {
				return position_three_states.DriveForward;
			}
			return null;
		}).addState(position_three_states.DriveForward, () -> {
			bot.robotInstruction.drive_left = 0.345;
			bot.robotInstruction.drive_right = 0.345;
			bot.robotInstruction.reset_gyro = true;
		}, () -> {
			if (((bot.robotState.drive_left_dist + bot.robotState.drive_left_dist) / 2.0) * 0.095 > 54) {
				return position_three_states.TurnLeft;
			}
			return null;
		}).addState(position_three_states.TurnLeft, () -> {
			if (Math.abs(bot.robotState.gyro) > -40) {
				bot.robotInstruction.drive_left = -0.4;
				bot.robotInstruction.drive_right = 0.4;
			} else {
				bot.robotInstruction.drive_left = 0.4;
				bot.robotInstruction.drive_right = -0.4;
			}
			bot.robotInstruction.reset_drive = true;
		}, () -> {
			if (Math.abs(bot.robotState.gyro + 40) < 1) {
				return position_three_states.DriveForwardTwo;
			}
			return null;
		}).addState(position_three_states.DriveForwardTwo, () -> {
			bot.robotInstruction.drive_left = 0.2;
			bot.robotInstruction.drive_right = 0.2;
		}, () -> {
			if (((bot.robotState.drive_left_dist + bot.robotState.drive_left_dist) / 2.0) * 0.095 > 111) {
				return position_three_states.Finished;
			}
			return null;
		}).addState(position_three_states.Finished, () -> {
			bot.robotInstruction.arm_high = true;
			bot.robotInstruction.fingers_open = true;
		}, () -> {
			return null;
		});

		position_four_sm.addState(position_four_states.TurnLeft, () -> {
			if (bot.robotState.gyro > -15) {
				bot.robotInstruction.drive_left = -0.4;
				bot.robotInstruction.drive_right = 0.4;
			} else {
				bot.robotInstruction.drive_left = 0.4;
				bot.robotInstruction.drive_right = -0.4;
			}
			bot.robotInstruction.reset_drive = true;
		}, () -> {
			if (Math.abs(bot.robotState.gyro + 15) < 1) {
				return position_four_states.DriveForward;
			}
			return null;
		}).addState(position_four_states.DriveForward, () -> {
			bot.robotInstruction.drive_left = 0.345;
			bot.robotInstruction.drive_right = 0.345;
			bot.robotInstruction.reset_gyro = true;
		}, () -> {
			if (((bot.robotState.drive_left_dist + bot.robotState.drive_left_dist) / 2.0) * 0.095 > 62) {
				return position_four_states.TurnRight;
			}
			return null;
		}).addState(position_four_states.TurnRight, () -> {
			if (bot.robotState.gyro > 15) {
				bot.robotInstruction.drive_left = -0.4;
				bot.robotInstruction.drive_right = 0.4;
			} else {
				bot.robotInstruction.drive_left = 0.4;
				bot.robotInstruction.drive_right = -0.4;
			}
			bot.robotInstruction.reset_drive = true;
		}, () -> {
			if (Math.abs(bot.robotState.gyro - 15) < 1) {
				return position_four_states.DriveForwardTwo;
			}
			return null;
		}).addState(position_four_states.DriveForwardTwo, () -> {
			bot.robotInstruction.drive_left = 0.2;
			bot.robotInstruction.drive_right = 0.2;
		}, () -> {
			if (((bot.robotState.drive_left_dist + bot.robotState.drive_left_dist) / 2.0) * 0.095 > 89) {
				return position_four_states.Finished;
			}
			return null;
		}).addState(position_four_states.Finished, () -> {
			bot.robotInstruction.arm_high = true;
			bot.robotInstruction.fingers_open = true;
		}, () -> {
			return null;
		});

		position_five_sm.addState(position_five_states.Center, () -> {
			if (bot.robotState.gyro > 0) {
				bot.robotInstruction.drive_left = -0.4;
				bot.robotInstruction.drive_right = 0.4;
			} else {
				bot.robotInstruction.drive_left = 0.4;
				bot.robotInstruction.drive_right = -0.4;
			}
			bot.robotInstruction.reset_drive = true;
		}, () -> {
			if (Math.abs(bot.robotState.gyro) < 1) {
				return position_five_states.DriveForward;
			}
			return null;
		}).addState(position_five_states.DriveForward, () -> {
			bot.robotInstruction.drive_left = 0.345;
			bot.robotInstruction.drive_right = 0.345;
			bot.robotInstruction.reset_gyro = true;
		}, () -> {
			if (((bot.robotState.drive_left_dist + bot.robotState.drive_left_dist) / 2.0) * 0.095 > 120) {
				return position_five_states.Turn;
			}
			return null;
		}).addState(position_five_states.Turn, () -> {
			if (bot.robotState.gyro > -55) {
				bot.robotInstruction.drive_left = -0.4;
				bot.robotInstruction.drive_right = 0.4;
			} else {
				bot.robotInstruction.drive_left = 0.4;
				bot.robotInstruction.drive_right = -0.4;
			}
			bot.robotInstruction.reset_drive = true;
		}, () -> {
			if (Math.abs(bot.robotState.gyro + 50) < 1) {
				return position_five_states.DriveForwardTwo;
			}
			return null;
		}).addState(position_five_states.DriveForwardTwo, () -> {
			bot.robotInstruction.drive_left = 0.2;
			bot.robotInstruction.drive_right = 0.2;
		}, () -> {
			if (((bot.robotState.drive_left_dist + bot.robotState.drive_left_dist) / 2.0) * 0.095 > 58) {
				return position_five_states.Finished;
			}
			return null;
		}).addState(position_five_states.Finished, () -> {
			bot.robotInstruction.arm_high = true;
			bot.robotInstruction.fingers_open = true;
		}, () -> {
			return null;
		});

		// FIXME
		portcullis_sm.addState(portcullis_states.Accelerate, () -> {
			// bot.robotInstruction.reset_defense = true; not needed if lowering defense at same time
			bot.robotInstruction.intake_down = true;
			bot.robotInstruction.defense_down = true;
			bot.robotInstruction.arm_auto = true;
			bot.robotInstruction.drive_left = 0.345;
			bot.robotInstruction.drive_right = 0.345;
		}, () -> {
			if (bot.robotState.arm_aligned_auto && bot.robotState.manip_encoder / 10.0 > 0.25) {
				// FIXME: counts per rotation
				return portcullis_states.ContinueDrive;
			}
			if (((bot.robotState.drive_left_dist + bot.robotState.drive_left_dist) / 2.0) * 0.095 > 21) {
				return portcullis_states.ContinueArmDefense;
			}
			return null;
		}).addState(portcullis_states.ContinueArmDefense, () -> {
			bot.robotInstruction.defense_down = true;
			bot.robotInstruction.arm_auto = true;
		}, () -> {
			if (bot.robotState.arm_aligned_auto && bot.robotState.manip_encoder / 10.0 > 0.25) {
				// FIXME: counts per rotation
				return portcullis_states.ArmLow;
			}
			return null;
		}).addState(portcullis_states.ContinueDrive, () -> {
			bot.robotInstruction.drive_left = 0.345;
			bot.robotInstruction.drive_right = 0.345;
		}, () -> {
			if (((bot.robotState.drive_left_dist + bot.robotState.drive_left_dist) / 2.0) * 0.095 > 21) {
				return portcullis_states.ArmLow;
			}
			return null;
		}).addState(portcullis_states.ArmLow, () -> {
			bot.robotInstruction.arm_low = true;
			bot.robotInstruction.drive_left = .345;
			bot.robotInstruction.drive_right = .345;
		}, () -> {
			if (((bot.robotState.drive_left_dist + bot.robotState.drive_left_dist) / 2.0) * 0.095 > 55) {
				return portcullis_states.Finished;
			} else
				return null;
		}).addState(portcullis_states.Finished, () -> {
			return null;
		});

		sally_port_sm.addState(sally_port_states.Accelerate, () -> {
			bot.robotInstruction.drive_left = 0.5 * ((System.currentTimeMillis() - bot.robotState.start_time) / 1000.0);
			bot.robotInstruction.drive_right = 0.5 * ((System.currentTimeMillis() - bot.robotState.start_time) / 1000.0);
		}, () -> {
			if (Math.abs(System.currentTimeMillis() - bot.robotState.start_time) > 1000)
				return sally_port_states.Finished;
			else
				return null;
		}).addState(sally_port_states.Finished, () -> {
			return null;
		});

		drawbridge_sm.addState(drawbridge_states.Accelerate, () -> {
			bot.robotInstruction.drive_left = 0.5 * ((System.currentTimeMillis() - bot.robotState.start_time) / 1000.0);
			bot.robotInstruction.drive_right = 0.5 * ((System.currentTimeMillis() - bot.robotState.start_time) / 1000.0);
		}, () -> {
			if (Math.abs(System.currentTimeMillis() - bot.robotState.start_time) > 1000)
				return drawbridge_states.Finished;
			else
				return null;
		}).addState(drawbridge_states.Finished, () -> {
			return null;
		});

		cheval_sm.addState(cheval_states.DriveToCheval, () -> {
			bot.robotInstruction.drive_left = 0.4;
			bot.robotInstruction.drive_right = 0.4;
		}, () -> {
			// TODO: Fix distance
			if (((bot.robotState.drive_left_dist + bot.robotState.drive_left_dist) / 2.0) * 0.095 > 21) {
				return cheval_states.DropIntake;
			} else
				return null;
		}).addState(cheval_states.DropIntake, () -> {
			bot.robotInstruction.intake_down = true;
		}, () -> {
			if (bot.robotState.intake_down) {
				return cheval_states.DropArmToCheval;
			} else
				return null;
		}).addState(cheval_states.DropArmToCheval, () -> {
			bot.robotInstruction.arm_cheval = true;
		}, () -> {
			if (bot.robotState.arm_aligned_cheval)
				return cheval_states.DriveAcrossCheval;
			else
				return null;
		}).addState(cheval_states.DriveAcrossCheval, () -> {
			bot.robotInstruction.drive_left = 0.3;
			bot.robotInstruction.drive_right = 0.3;
		}, () -> {
			if (System.currentTimeMillis() - bot.robotState.start_time > 3000)
				return cheval_states.Finished;
			return null;
		}).addState(cheval_states.Finished, () -> {
			return null;
		});
	}

	private long last;

	public void teleopInit() {
		shoot_sm.setAuto(false);
		// Makes sure the current shoot_sm state isn't null, because it starts out as null and if autoInit isn't called it remains unitialized,
		// causing a null pointer exception (what I think the soucre of the error is)
		if (shoot_sm.getState() == null || shoot_sm.getState() == shoot_states.AutoInit)
			shoot_sm.setState(shoot_states.WaitToShoot);
	}

	public void teleopPeriodic() {
		if (shoot_sm.getState().equals(shoot_states.AutoInit)) {
			shoot_sm.setState(shoot_states.WaitToShoot);
		}
		if (shoot_sm.getState().equals(shoot_states.PointAtGoal)) {
			bot.getCameraState();
		} else {
			bot.getState(input.getShoot(), input.getResetShot());
		}
		shoot_sm.process();
		bot.run(input);
		SmartDashboard.putNumber("Delay", System.currentTimeMillis() - last);
		last = System.currentTimeMillis();
	}

	public void testInit() {
		System.out.println("Test Initialization");
	}

	public void testPeriodic() {
		bot.test_mode(input);
	}

	public void disabledPeriodic() {
		bot.disabled();
		SmartDashboard.putNumber("Delay", System.currentTimeMillis() - last);
		last = System.currentTimeMillis();
	}

	final String do_nothing = "Do Nothing";
	final String cross_defenses = "Cross Defenses";
	final String approach_defenses = "Approach Defenses";
	String start_mode;
	SendableChooser start_chooser = new SendableChooser();

	final String cross_terrain = "Cross Terrain";
	final String portcullis = "Portcullis";
	final String cheval = "Cheval de Frise";
	final String drawbridge = "Drawbridge";
	final String sally_port = "Sally Port";
	final String low_bar = "Low Bar";
	String defense;
	SendableChooser defense_chooser = new SendableChooser();

	final String position_one = "1";
	final String position_two = "2";
	final String position_three = "3";
	final String position_four = "4";
	final String position_five = "5";
	String position;
	SendableChooser position_chooser = new SendableChooser();

	boolean over_defenses = false;
	boolean ready_to_shoot = false;

	public void autonomousInit() {
		over_defenses = false;
		start_mode = (String) start_chooser.getSelected();
		defense = (String) defense_chooser.getSelected();
		position = (String) position_chooser.getSelected();

		cross_terrain_sm.setState(cross_terrain_states.ArmDown);
		approach_sm.setState(approach_states.Accelerate);
		low_bar_sm.setState(low_bar_states.DropIntake);
		portcullis_sm.setState(portcullis_states.Accelerate);
		sally_port_sm.setState(sally_port_states.Accelerate);
		drawbridge_sm.setState(drawbridge_states.Accelerate);
		cheval_sm.setState(cheval_states.DriveToCheval);

		position_two_sm.setState(position_two_states.Center);
		position_three_sm.setState(position_three_states.TurnRight);
		position_four_sm.setState(position_four_states.TurnLeft);
		position_five_sm.setState(position_five_states.Center);

		shoot_sm.setAuto(true);
		shoot_sm.setState(shoot_states.AutoInit);

		bot.prepareAuto();
	}

	public void autonomousPeriodic() {
		bot.getAutoState();
		if (start_mode.equals(do_nothing)) {
			// Do nothing
		} else if (start_mode.equals(approach_defenses)) {
			approach_sm.process(); // approach the batter
		} else if (start_mode.equals(cross_defenses)) {
			if (!over_defenses) {
				if (defense.equals(do_nothing)) {
					// Do nothing
				} else if (defense.equals(cross_terrain)) {
					cross_terrain_sm.process();
					if (cross_terrain_sm.getState().equals(cross_terrain_states.Finished))
						over_defenses = true;
				} else if (defense.equals(cheval)) {
					cheval_sm.process();
					if (cheval_sm.getState().equals(cheval_states.Finished))
						over_defenses = true;
				} else if (defense.equals(low_bar)) {
					low_bar_sm.process();
					if (low_bar_sm.equals(low_bar_states.Finished))
						over_defenses = true;
				} else if (defense.equals(drawbridge)) {
					drawbridge_sm.process();
					if (drawbridge_sm.getState().equals(drawbridge_states.Finished))
						over_defenses = true;
				} else if (defense.equals(portcullis)) {
					portcullis_sm.process();
					if (portcullis_sm.getState().equals(portcullis_states.Finished))
						over_defenses = true;
				} else if (defense.equals(sally_port)) {
					sally_port_sm.process();
					if (sally_port_sm.getState().equals(sally_port_states.Finished))
						over_defenses = true;
				} else {
					System.err.println("Unknown Crossing Auto Mode");
					// do nothing
				}
			} else if (!ready_to_shoot) { // crossing state machine has finished, not ready to shoot
				if (position.equals(position_one)) {
					if (low_bar_sm.getState().equals(low_bar_states.Finished))
						ready_to_shoot = false;
				} else if (position.equals(position_two)) {
					position_two_sm.process();
					if (position_two_sm.getState().equals(position_two_states.Finished))
						ready_to_shoot = false;
				} else if (position.equals(position_three)) {
					position_three_sm.process();
					if (position_three_sm.getState().equals(position_three_states.Finished))
						ready_to_shoot = false;
				} else if (position.equals(position_four)) {
					position_four_sm.process();
					if (position_four_sm.getState().equals(position_four_states.Finished))
						ready_to_shoot = false;
				} else if (position.equals(position_five)) {
					position_five_sm.process();
					if (position_five_sm.getState().equals(position_five_states.Finished))
						ready_to_shoot = false;
				} else if (position.equals(do_nothing)) {
					// Do nothing after crossing
				} else {
					System.err.println("Unknown Posistion Auto Mode");
					// Do nothing
				}
			} else if (ready_to_shoot) {
				if(shoot_sm.getState().equals(shoot_states.PointAtGoal)) {
					bot.getCameraState();
				}
				shoot_sm.process(); // Shoot then stop (should proceed to wait to shoot state after shooting)
			} else {
				// Do nothing (if finished with crossing and position and done shooting)
			}
		} else {
			// Do nothing, no mode was selected/some sort of error occured
			System.err.println("Mode not selected");
		}
		bot.run();
	}
}