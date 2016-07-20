
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

	Systems bot;
	Input input = new Input();

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
		Accelerate, Drive, LowMans, DriveTwo, RaiseMans, Finished;
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

		shoot_sm.addState(shoot_states.WaitToShoot, (RobotState rbs) -> {
			return new RobotInstruction<shoot_states>();
		}, (RobotState rbs) -> {
			if (rbs.shoot && rbs.fingers_open)
				return shoot_states.ShootMode;
			if (rbs.reset_catapult && rbs.fingers_open)
				return shoot_states.ShootMode;
			/*
			 * else if (rbs.shoot) return "Open Fingers";
			 */
			else
				return null;
		}).addState(shoot_states.AutoInit, (RobotState rbs) -> {
			return new RobotInstruction<shoot_states>();
		}, (RobotState rbs) -> {
			if (rbs.arm_aligned_high)
				return shoot_states.ShootMode;
			else
				return null;
		}).addState(shoot_states.ShootMode, (RobotState rbs) -> {
			return new RobotInstruction<shoot_states>();
		}, (RobotState rbs) -> {
			if (rbs.shoot_mode_far || rbs.shoot_mode_close)
				return shoot_states.ShootPosition;
			else if (!rbs.camera_exists)
				return shoot_states.WaitToShoot;
			else if (rbs.shoot_mode_auto)
				return shoot_states.PointAtGoal;
			else
				return null;
		}).addState(shoot_states.PointAtGoal, (RobotState rbs) -> {
			RobotInstruction<shoot_states> rbi = new RobotInstruction<shoot_states>();
			if (rbs.angle_to_goal == -1.0) {
				rbi.drive_left = 0.2;
				rbi.drive_right = -0.2;
			}
			double turn = -0.2;
			if (rbs.angle_to_goal < 0.5)
				turn = 0.2;
			else
				turn = -0.2;
			rbi.drive_auto = true;
			rbi.drive_left = turn;
			rbi.drive_right = -turn;
			return rbi;
		}, (RobotState rbs) -> {
			// FIXME: change this after testing turning
			if (Math.abs(0.5 - rbs.angle_to_goal) < 0.1)
				return shoot_states.WaitToShoot;
			else if (!rbs.shoot_mode_auto)
				return shoot_states.WaitToShoot;
			else
				return null;
		}).addState(shoot_states.AutoShootPosition, (RobotState rbs) -> {
			RobotInstruction<shoot_states> rbi = new RobotInstruction<shoot_states>();
			// TODO: calculate function for setpoint based on distance
			//rbi.catapult_auto_pos = (int) (rbs.distance_to_goal * 2);
			rbi.catapult_shoot = true;
			return rbi;
		}, (RobotState rbs) -> {
			if (rbs.catapult_aligned_shoot && rbs.arm_aligned_high && rbs.fingers_open
					&& ((Math.abs(System.currentTimeMillis() - rbs.start_time) > 500)))
				return shoot_states.Shoot;
			else
				return null;
		}).addState(shoot_states.ShootPosition, (RobotState rbs) -> {
			RobotInstruction<shoot_states> rbi = new RobotInstruction<shoot_states>();
			rbi.catapult_shoot = true;
			return rbi;
		}, (RobotState rbs) -> {
			if (rbs.catapult_aligned_shoot && rbs.arm_aligned_high && rbs.fingers_open
					&& ((Math.abs(System.currentTimeMillis() - rbs.start_time) > 500)))
				return shoot_states.Shoot;
			if (rbs.reset_catapult && rbs.fingers_open)
				return shoot_states.Shoot;
			else
				return null;
		})/*
			 * .addState(new State("Open Fingers", (RobotState rbs) -> { RobotInstruction rbi = new RobotInstruction(); // rbi.fingers_open = true;
			 * return rbi; } , (RobotState rbs) -> { //if ((Math.abs(System.currentTimeMillis() - rbs.start_time) > 200)) return "Shoot Position";
			 * //else // return null; }))
			 */.addState(shoot_states.Shoot, (RobotState rbs) -> {
			RobotInstruction<shoot_states> rbi = new RobotInstruction<shoot_states>();
			rbi.catapult_release = true;
			return rbi;
		}, (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 200)
				return shoot_states.RetrieveTram;
			else
				return null;
		}).addState(shoot_states.RetrieveTram, (RobotState rbs) -> {
			RobotInstruction<shoot_states> rbi = new RobotInstruction<shoot_states>();
			rbi.catapult_out = true;
			return rbi;
		}, (RobotState rbs) -> {
			if (rbs.catapult_aligned_out)
				return shoot_states.LatchTram;
			else
				return null;
		}).addState(shoot_states.LatchTram, (RobotState rbs) -> {
			RobotInstruction<shoot_states> rbi = new RobotInstruction<shoot_states>();
			rbi.catapult_latch = true;
			return rbi;
		}, (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 500)
				return shoot_states.PullTram;
			else
				return null;
		}).addState(shoot_states.PullTram, (RobotState rbs) -> {
			RobotInstruction<shoot_states> rbi = new RobotInstruction<shoot_states>();
			rbi.catapult_load = true;
			return rbi;
		}, (RobotState rbs) -> {
			if (rbs.catapult_aligned_load)
				return shoot_states.WaitToShoot;
			else
				return null;
		});

		cross_terrain_sm.addState(cross_terrain_states.ArmDown, (RobotState rbs) -> {
			RobotInstruction<cross_terrain_states> rbi = new RobotInstruction<cross_terrain_states>();
			rbi.intake_down = true;
			rbi.arm_middle = true;
			return rbi;
		}, (RobotState rbs) -> { // FIXME?
			if (rbs.arm_aligned_middle)
				return cross_terrain_states.DriveAcc;
			else
				return null;
		}).addState(cross_terrain_states.DriveAcc, (RobotState rbs) -> {
			RobotInstruction<cross_terrain_states> rbi = new RobotInstruction<cross_terrain_states>();
			rbi.drive_left = 0.8 * ((System.currentTimeMillis() - rbs.start_time) / 2500.0);
			rbi.drive_right = 0.8 * ((System.currentTimeMillis() - rbs.start_time) / 2500.0);
			return rbi;
		}, (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 2500)
				return cross_terrain_states.DriveSteady;
			else
				return null;
		}).addState(cross_terrain_states.DriveSteady, (RobotState rbs) -> {
			RobotInstruction<cross_terrain_states> rbi = new RobotInstruction<cross_terrain_states>();
			rbi.drive_left = 0.8 - (0.8 * ((System.currentTimeMillis() - rbs.start_time) / 3000.0));
			rbi.drive_right = 0.8 - (0.8 * ((System.currentTimeMillis() - rbs.start_time) / 3000.0));
			return rbi;
		}, (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 3000)
				return cross_terrain_states.Finished;
			else
				return null;
		}).addState(cross_terrain_states.Finished, (RobotState rbs) -> {
			RobotInstruction<cross_terrain_states> rbi = new RobotInstruction<cross_terrain_states>();
			return rbi;
		}, (RobotState rbs) -> {
			return null;
		});

		approach_sm.addState(approach_states.Accelerate, (RobotState rbs) -> {
			RobotInstruction<approach_states> rbi = new RobotInstruction<approach_states>();
			rbi.drive_left = 0.5 * ((System.currentTimeMillis() - rbs.start_time) / 1000.0);
			rbi.drive_right = 0.5 * ((System.currentTimeMillis() - rbs.start_time) / 1000.0);
			return rbi;
		}, (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 1000)
				return approach_states.Finished;
			else
				return null;
		}).addState(approach_states.Finished, (RobotState rbs) -> {
			RobotInstruction<approach_states> rbi = new RobotInstruction<approach_states>();
			return rbi;
		}, (RobotState rbs) -> {
			return null;
		});

		low_bar_sm.addState(low_bar_states.DropIntake, (RobotState rbs) -> {
			RobotInstruction<low_bar_states> rbi = new RobotInstruction<low_bar_states>();
			rbi.intake_down = true;
			return rbi;
		}, (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 1000)
				return low_bar_states.DropArm;
			else
				return null;
		}).addState(low_bar_states.DropArm, (RobotState rbs) -> {
			RobotInstruction<low_bar_states> rbi = new RobotInstruction<low_bar_states>();
			rbi.catapult_load = true;
			rbi.fingers_close = true;
			if (rbs.catapult_aligned_load && rbs.fingers_closed && rbs.intake_down) {
				rbi.arm_low = true;
			}
			return rbi;
		}, (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 1000)
				return low_bar_states.Wait;
			else
				return null;
		}).addState(low_bar_states.Wait, (RobotState rbs) -> {
			RobotInstruction<low_bar_states> rbi = new RobotInstruction<low_bar_states>();
			return rbi;
		}, (RobotState rbs) -> {
			if (rbs.arm_aligned_low)
				return low_bar_states.DriveForward;
			else
				return null;
		}).addState(low_bar_states.DriveForward, (RobotState rbs) -> {
			RobotInstruction<low_bar_states> rbi = new RobotInstruction<low_bar_states>();
			rbi.reset_gyro = true;
			rbi.drive_left = 0.345;
			rbi.drive_right = 0.345;
			return rbi;
		}, (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 220) {
				return low_bar_states.Turn;
			}
			return null;
		}).addState(low_bar_states.Turn, (RobotState rbs) -> {
			RobotInstruction<low_bar_states> rbi = new RobotInstruction<low_bar_states>();
			if (rbs.gyro > 55) {
				rbi.drive_left = -0.4;
				rbi.drive_right = 0.4;
			} else {
				rbi.drive_left = 0.4;
				rbi.drive_right = -0.4;
			}
			rbi.reset_drive = true;
			return rbi;
		}, (RobotState rbs) -> {
			if (Math.abs(rbs.gyro - 55) < 1) {
				return low_bar_states.DriveForwardTwo;
			}
			return null;
		}).addState(low_bar_states.DriveForwardTwo, (RobotState rbs) -> {
			RobotInstruction<low_bar_states> rbi = new RobotInstruction<low_bar_states>();
			rbi.drive_left = 0.2;
			rbi.drive_right = 0.2;
			return rbi;
		}, (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 160) {
				return low_bar_states.RaiseArm;
			}
			return null;
		}).addState(low_bar_states.RaiseArm, (RobotState rbs) -> {
			RobotInstruction<low_bar_states> rbi = new RobotInstruction<low_bar_states>();
			rbi.arm_high = true;
			return rbi;
		}, (RobotState rbs) -> {
			if (System.currentTimeMillis() - rbs.start_time > 500)
				return low_bar_states.OpenFingers;
			else
				return null;
		}).addState(low_bar_states.OpenFingers, (RobotState rbs) -> {
			RobotInstruction<low_bar_states> rbi = new RobotInstruction<low_bar_states>();
			rbi.fingers_open = true;
			return rbi;
		}, (RobotState rbs) -> {
			if (System.currentTimeMillis() - rbs.start_time > 500)
				return low_bar_states.Finished;
			else
				return null;
		}).addState(low_bar_states.Finished, (RobotState rbs) -> {
			return new RobotInstruction<low_bar_states>();
		}, (RobotState rbs) -> {
			return null;
		});

		position_two_sm.addState(position_two_states.Center, (RobotState rbs) -> {
			RobotInstruction<position_two_states> rbi = new RobotInstruction<position_two_states>();
			if (rbs.gyro > 0) {
				rbi.drive_left = -0.4;
				rbi.drive_right = 0.4;
			} else {
				rbi.drive_left = 0.4;
				rbi.drive_right = -0.4;
			}
			rbi.reset_drive = true;
			return rbi;
		}, (RobotState rbs) -> {
			if (Math.abs(rbs.gyro) < 1) {
				return position_two_states.DriveForward;
			}
			return null;
		}).addState(position_two_states.DriveForward, (RobotState rbs) -> {
			RobotInstruction<position_two_states> rbi = new RobotInstruction<position_two_states>();
			/*
			 * if (dist < 5) { drive_speed = 0.3; } else if (dist < 30) { drive_speed = 0.6 * (dist) / 30.0; } else if (dist > 90) { drive_speed =
			 * -1/30.0 * dist + (18/5.0); } else { drive_speed = 0.6; }
			 */
			rbi.reset_gyro = true;
			rbi.drive_left = 0.345;
			rbi.drive_right = 0.345;
			return rbi;
		}, (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 105) {
				return position_two_states.Turn;
			}
			return null;
		}).addState(position_two_states.Turn, (RobotState rbs) -> {
			RobotInstruction<position_two_states> rbi = new RobotInstruction<position_two_states>();
			if (rbs.gyro > +55) {
				rbi.drive_left = -0.4;
				rbi.drive_right = 0.4;
			} else {
				rbi.drive_left = 0.4;
				rbi.drive_right = -0.4;
			}
			rbi.reset_drive = true;
			return rbi;
		}, (RobotState rbs) -> {
			if (Math.abs(rbs.gyro - 55) < 1) {
				return position_two_states.DriveForwardTwo;
			}
			return null;
		}).addState(position_two_states.DriveForwardTwo, (RobotState rbs) -> {
			RobotInstruction<position_two_states> rbi = new RobotInstruction<position_two_states>();
			rbi.drive_left = 0.2;
			rbi.drive_right = 0.2;
			return rbi;
		}, (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 53) {
				return position_two_states.Finished;
			}
			return null;
		}).addState(position_two_states.Finished, (RobotState rbs) -> {
			RobotInstruction<position_two_states> rbi = new RobotInstruction<position_two_states>();
			rbi.arm_high = true;
			return rbi;
		}, (RobotState rbs) -> {
			return null;
		});

		position_three_sm.addState(position_three_states.TurnRight, (RobotState rbs) -> {
			RobotInstruction<position_three_states> rbi = new RobotInstruction<position_three_states>();
			if (rbs.gyro > 40) {
				rbi.drive_left = -0.4;
				rbi.drive_right = 0.4;
			} else {
				rbi.drive_left = 0.4;
				rbi.drive_right = -0.4;
			}
			rbi.reset_drive = true;
			return rbi;
		}, (RobotState rbs) -> {
			if (Math.abs(rbs.gyro - 40) < 1) {
				return position_three_states.DriveForward;
			}
			return null;
		}).addState(position_three_states.DriveForward, (RobotState rbs) -> {
			RobotInstruction<position_three_states> rbi = new RobotInstruction<position_three_states>();

			/*
			 * if (dist < 5) { drive_speed = 0.3; } else if (dist < 30) { drive_speed = 0.6 * (dist) / 30.0; } else if (dist > 90) { drive_speed =
			 * -1/30.0 * dist + (18/5.0); } else { drive_speed = 0.6; }
			 */

			rbi.drive_left = 0.345;
			rbi.drive_right = 0.345;
			rbi.reset_gyro = true;
			return rbi;
		}, (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 54) {
				return position_three_states.TurnLeft;
			}
			return null;
		}).addState(position_three_states.TurnLeft, (RobotState rbs) -> {
			RobotInstruction<position_three_states> rbi = new RobotInstruction<position_three_states>();
			if (Math.abs(rbs.gyro) > -40) {
				rbi.drive_left = -0.4;
				rbi.drive_right = 0.4;
			} else {
				rbi.drive_left = 0.4;
				rbi.drive_right = -0.4;
			}
			rbi.reset_drive = true;
			return rbi;
		}, (RobotState rbs) -> {
			if (Math.abs(rbs.gyro + 40) < 1) {
				return position_three_states.DriveForwardTwo;
			}
			return null;
		}).addState(position_three_states.DriveForwardTwo, (RobotState rbs) -> {
			RobotInstruction<position_three_states> rbi = new RobotInstruction<position_three_states>();

			/*
			 * if (dist < 5) { drive_speed = 0.3; } else if (dist < 30) { drive_speed = 0.6 * (dist) / 30.0; } else if (dist > 90) { drive_speed =
			 * -1/30.0 * dist + (18/5.0); } else { drive_speed = 0.6; }
			 */

			rbi.drive_left = 0.2;
			rbi.drive_right = 0.2;
			return rbi;
		}, (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 111) {
				return position_three_states.Finished;
			}
			return null;
		}).addState(position_three_states.Finished, (RobotState rbs) -> {
			RobotInstruction<position_three_states> rbi = new RobotInstruction<position_three_states>();
			rbi.arm_high = true;
			return rbi;
		}, (RobotState rbs) -> {
			return null;
		});

		position_four_sm.addState(position_four_states.TurnLeft, (RobotState rbs) -> {
			RobotInstruction<position_four_states> rbi = new RobotInstruction<position_four_states>();
			if (rbs.gyro > -15) {
				rbi.drive_left = -0.4;
				rbi.drive_right = 0.4;
			} else {
				rbi.drive_left = 0.4;
				rbi.drive_right = -0.4;
			}
			rbi.reset_drive = true;
			return rbi;
		}, (RobotState rbs) -> {
			if (Math.abs(rbs.gyro + 15) < 1) {
				return position_four_states.DriveForward;
			}
			return null;
		}).addState(position_four_states.DriveForward, (RobotState rbs) -> {
			RobotInstruction<position_four_states> rbi = new RobotInstruction<position_four_states>();

			/*
			 * if (dist < 5) { drive_speed = 0.3; } else if (dist < 30) { drive_speed = 0.6 * (dist) / 30.0; } else if (dist > 90) { drive_speed =
			 * -1/30.0 * dist + (18/5.0); } else { drive_speed = 0.6; }
			 */

			rbi.drive_left = 0.345;
			rbi.drive_right = 0.345;
			rbi.reset_gyro = true;
			return rbi;
		}, (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 62) {
				return position_four_states.TurnRight;
			}
			return null;
		}).addState(position_four_states.TurnRight, (RobotState rbs) -> {
			RobotInstruction<position_four_states> rbi = new RobotInstruction<position_four_states>();
			if (rbs.gyro > 15) {
				rbi.drive_left = -0.4;
				rbi.drive_right = 0.4;
			} else {
				rbi.drive_left = 0.4;
				rbi.drive_right = -0.4;
			}
			rbi.reset_drive = true;
			return rbi;
		}, (RobotState rbs) -> {
			if (Math.abs(rbs.gyro - 15) < 1) {
				return position_four_states.DriveForwardTwo;
			}
			return null;
		}).addState(position_four_states.DriveForwardTwo, (RobotState rbs) -> {
			RobotInstruction<position_four_states> rbi = new RobotInstruction<position_four_states>();
			/*
			 * if (dist < 5) { drive_speed = 0.3; } else if (dist < 30) { drive_speed = 0.6 * (dist) / 30.0; } else if (dist > 90) { drive_speed =
			 * -1/30.0 * dist + (18/5.0); } else { drive_speed = 0.6; }
			 */

			rbi.drive_left = 0.2;
			rbi.drive_right = 0.2;
			return rbi;
		}, (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 89) {
				return position_four_states.Finished;
			}
			return null;
		}).addState(position_four_states.Finished, (RobotState rbs) -> {
			RobotInstruction<position_four_states> rbi = new RobotInstruction<position_four_states>();
			rbi.arm_high = true;
			return rbi;
		}, (RobotState rbs) -> {
			return null;
		});

		position_five_sm.addState(position_five_states.Center, (RobotState rbs) -> {
			RobotInstruction<position_five_states> rbi = new RobotInstruction<position_five_states>();
			if (rbs.gyro > 0) {
				rbi.drive_left = -0.4;
				rbi.drive_right = 0.4;
			} else {
				rbi.drive_left = 0.4;
				rbi.drive_right = -0.4;
			}
			rbi.reset_drive = true;
			return rbi;
		}, (RobotState rbs) -> {
			if (Math.abs(rbs.gyro) < 1) {
				return position_five_states.DriveForward;
			}
			return null;
		}).addState(position_five_states.DriveForward, (RobotState rbs) -> {
			RobotInstruction<position_five_states> rbi = new RobotInstruction<position_five_states>();
			/*
			 * if (dist < 5) { drive_speed = 0.3; } else if (dist < 30) { drive_speed = 0.6 * (dist) / 30.0; } else if (dist > 90) { drive_speed =
			 * -1/30.0 * dist + (18/5.0); } else { drive_speed = 0.6; }
			 */

			rbi.drive_left = 0.345;
			rbi.drive_right = 0.345;
			rbi.reset_gyro = true;
			return rbi;
		}, (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 120) {
				return position_five_states.Turn;
			}
			return null;
		}).addState(position_five_states.Turn, (RobotState rbs) -> {
			RobotInstruction<position_five_states> rbi = new RobotInstruction<position_five_states>();
			if (rbs.gyro > -55) {
				rbi.drive_left = -0.4;
				rbi.drive_right = 0.4;
			} else {
				rbi.drive_left = 0.4;
				rbi.drive_right = -0.4;
			}
			rbi.reset_drive = true;
			return rbi;
		}, (RobotState rbs) -> {
			if (Math.abs(rbs.gyro + 50) < 1) {
				return position_five_states.DriveForwardTwo;
			}
			return null;
		}).addState(position_five_states.DriveForwardTwo, (RobotState rbs) -> {
			RobotInstruction<position_five_states> rbi = new RobotInstruction<position_five_states>();

			/*
			 * if (dist < 5) { drive_speed = 0.3; } else if (dist < 30) { drive_speed = 0.6 * (dist) / 30.0; } else if (dist > 90) { drive_speed =
			 * -1/30.0 * dist + (18/5.0); } else { drive_speed = 0.6; }
			 */

			rbi.drive_left = 0.2;
			rbi.drive_right = 0.2;
			return rbi;
		}, (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 58) {
				return position_five_states.Finished;
			}
			return null;
		}).addState(position_five_states.Finished, (RobotState rbs) -> {
			RobotInstruction<position_five_states> rbi = new RobotInstruction<position_five_states>();
			rbi.arm_high = true;
			return rbi;
		}, (RobotState rbs) -> {
			return null;
		});

		// FIXME
		portcullis_sm.addState(portcullis_states.Accelerate, (RobotState rbs) -> {
			RobotInstruction<portcullis_states> rbi = new RobotInstruction<portcullis_states>();
			// rbi.reset_defense = true; not needed if lowering defense at same time
			rbi.intake_down = true;
			rbi.defense_down = true;
			rbi.arm_auto = true;
			rbi.drive_left = 0.345;
			rbi.drive_right = 0.345;
			return rbi;
		}, (RobotState rbs) -> {
			if (rbs.arm_aligned_auto && rbs.manip_encoder / 10.0 > 0.25) {
				// FIXME: counts per rotation
				return portcullis_states.ContinueDrive;
			}
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 21) {
				return portcullis_states.ContinueArmDefense;
			}
			return null;
		}).addState(portcullis_states.ContinueArmDefense, (RobotState rbs) -> {
			RobotInstruction<portcullis_states> rbi = new RobotInstruction<portcullis_states>();
			rbi.defense_down = true;
			rbi.arm_auto = true;
			return rbi;
		}, (RobotState rbs) -> {
			if (rbs.arm_aligned_auto && rbs.manip_encoder / 10.0 > 0.25) {
				// FIXME: counts per rotation
				return portcullis_states.ArmLow;
			}
			return null;
		}).addState(portcullis_states.ContinueDrive, (RobotState rbs) -> {
			RobotInstruction<portcullis_states> rbi = new RobotInstruction<portcullis_states>();
			rbi.drive_left = 0.345;
			rbi.drive_right = 0.345;
			return rbi;
		}, (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 21) {
				return portcullis_states.ArmLow;
			}
			return null;
		}).addState(portcullis_states.ArmLow, (RobotState rbs) -> {
			RobotInstruction<portcullis_states> rbi = new RobotInstruction<portcullis_states>();
			rbi.arm_low = true;
			rbi.drive_left = .345;
			rbi.drive_right = .345;
			return rbi;
		}, (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 55) {
				return portcullis_states.Finished;
			} else
				return null;
		}).addState(portcullis_states.Finished, (RobotState rbs) -> {
			return new RobotInstruction<portcullis_states>();
		}, (RobotState rbs) -> {
			return null;
		});

		sally_port_sm.addState(sally_port_states.Accelerate, (RobotState rbs) -> {
			RobotInstruction<sally_port_states> rbi = new RobotInstruction<sally_port_states>();
			rbi.drive_left = 0.5 * ((System.currentTimeMillis() - rbs.start_time) / 1000.0);
			rbi.drive_right = 0.5 * ((System.currentTimeMillis() - rbs.start_time) / 1000.0);
			return rbi;
		}, (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 1000)
				return sally_port_states.Finished;
			else
				return null;
		}).addState(sally_port_states.Finished, (RobotState rbs) -> {
			RobotInstruction<sally_port_states> rbi = new RobotInstruction<sally_port_states>();
			return rbi;
		}, (RobotState rbs) -> {
			return null;
		});

		drawbridge_sm.addState(drawbridge_states.Accelerate, (RobotState rbs) -> {
			RobotInstruction<drawbridge_states> rbi = new RobotInstruction<drawbridge_states>();
			rbi.drive_left = 0.5 * ((System.currentTimeMillis() - rbs.start_time) / 1000.0);
			rbi.drive_right = 0.5 * ((System.currentTimeMillis() - rbs.start_time) / 1000.0);
			return rbi;
		}, (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 1000)
				return drawbridge_states.Finished;
			else
				return null;
		}).addState(drawbridge_states.Finished, (RobotState rbs) -> {
			return new RobotInstruction<drawbridge_states>();
		}, (RobotState rbs) -> {
			return null;
		});

		cheval_sm.addState(cheval_states.Accelerate, (RobotState rbs) -> {
			RobotInstruction<cheval_states> rbi = new RobotInstruction<cheval_states>();
			// rbi.drive_left = 0.5 * ((System.currentTimeMillis() - rbs.start_time) / 1000.0);
			// rbi.drive_right = 0.5 * ((System.currentTimeMillis() - rbs.start_time) / 1000.0);
			rbi.arm_auto = true;
			return rbi;
		}, (RobotState rbs) -> {
			if (rbs.arm_aligned_auto)
				return cheval_states.Drive;
			else
				return null;
		}).addState(cheval_states.Drive, (RobotState rbs) -> {
			RobotInstruction<cheval_states> rbi = new RobotInstruction<cheval_states>();
			rbi.drive_left = .345;
			rbi.drive_right = .345;
			return rbi;
		}, (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 21) {
				return cheval_states.LowMans;
			} else
				return null;
		}).addState(cheval_states.LowMans, (RobotState rbs) -> {
			RobotInstruction<cheval_states> rbi = new RobotInstruction<cheval_states>();
			rbi.defense_down = true;
			return rbi;
		}, (RobotState rbs) -> {
			if (rbs.manip_encoder / 10.0 > 0.25)
				return cheval_states.DriveTwo;
			return null;
		}).addState(cheval_states.DriveTwo, (RobotState rbs) -> {
			RobotInstruction<cheval_states> rbi = new RobotInstruction<cheval_states>();
			rbi.drive_left = .345;
			rbi.drive_right = .345;
			rbi.arm_middle = true;
			rbi.reset_defense = true;
			return rbi;
		}, (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 55) {
				return cheval_states.RaiseMans;
			} else
				return null;
		}).addState(cheval_states.RaiseMans, (RobotState rbs) -> {
			RobotInstruction<cheval_states> rbi = new RobotInstruction<cheval_states>();
			rbi.defense_up = true;
			return rbi;
		}, (RobotState rbs) -> {
			// FIXME: determine positive and negative direction on encoder for manipulaters
			if (rbs.manip_encoder / 10.0 < -0.25)
				return cheval_states.Finished;
			return null;
		}).addState(cheval_states.Finished, (RobotState rbs) -> {
			RobotInstruction<cheval_states> rbi = new RobotInstruction<cheval_states>();
			return rbi;
		}, (RobotState rbs) -> {
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
			bot.run(shoot_sm.process(bot.getCameraState()), input);
		} else {
			bot.run(shoot_sm.process(bot.getState(input.getShoot(), input.getResetShot())), input);
		}
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
		cheval_sm.setState(cheval_states.Accelerate);

		position_two_sm.setState(position_two_states.Center);
		position_three_sm.setState(position_three_states.TurnRight);
		position_four_sm.setState(position_four_states.TurnLeft);
		position_five_sm.setState(position_five_states.Center);

		shoot_sm.setAuto(true);
		shoot_sm.setState(shoot_states.AutoInit);

		bot.prepareAuto();
	}

	public void autonomousPeriodic() {
		if (start_mode.equals(do_nothing)) {
			bot.run(new RobotInstruction()); // Do nothing
		} else if (start_mode.equals(approach_defenses)) {
			bot.run(approach_sm.process(bot.getState())); // approach_sm batter
		} else if (start_mode.equals(cross_defenses)) {
			if (!over_defenses) {
				if (defense.equals(do_nothing)) {
					bot.run(new RobotInstruction()); // Do nothing
				} else if (defense.equals(cross_terrain)) {
					bot.run(cross_terrain_sm.process(bot.getState()));
					if (cross_terrain_sm.getState().equals(cross_terrain_states.Finished))
						over_defenses = true;
				} else if (defense.equals(cheval)) {
					bot.run(cheval_sm.process(bot.getState()));
					if (cheval_sm.getState().equals(cheval_states.Finished))
						over_defenses = true;
				} else if (defense.equals(low_bar)) {
					bot.run(low_bar_sm.process(bot.getState()));
					if (low_bar_sm.equals(low_bar_states.Finished))
						over_defenses = true;
				} else if (defense.equals(drawbridge)) {
					bot.run(drawbridge_sm.process(bot.getState()));
					if (drawbridge_sm.getState().equals(drawbridge_states.Finished))
						over_defenses = true;
				} else if (defense.equals(portcullis)) {
					bot.run(portcullis_sm.process(bot.getState()));
					if (portcullis_sm.getState().equals(portcullis_states.Finished))
						over_defenses = true;
				} else if (defense.equals(sally_port)) {
					bot.run(sally_port_sm.process(bot.getState()));
					if (sally_port_sm.getState().equals(sally_port_states.Finished))
						over_defenses = true;
				} else {
					System.err.println("Unknown Crossing Auto Mode");
					bot.run(new RobotInstruction()); // do nothing
				}
			} else if (!ready_to_shoot) { // crossing state machine has finished, not ready to shoot
				if (position.equals(position_one)) {
					if (low_bar_sm.getState().equals(low_bar_states.Finished))
						ready_to_shoot = false;
				} else if (position.equals(position_two)) {
					bot.run(position_two_sm.process(bot.getState()));
					if (position_two_sm.getState().equals(position_two_states.Finished))
						ready_to_shoot = false;
				} else if (position.equals(position_three)) {
					bot.run(position_three_sm.process(bot.getState()));
					if (position_three_sm.getState().equals(position_three_states.Finished))
						ready_to_shoot = false;
				} else if (position.equals(position_four)) {
					bot.run(position_four_sm.process(bot.getState()));
					if (position_four_sm.getState().equals(position_four_states.Finished))
						ready_to_shoot = false;
				} else if (position.equals(position_five)) {
					bot.run(position_five_sm.process(bot.getState()));
					if (position_five_sm.getState().equals(position_five_states.Finished))
						ready_to_shoot = false;
				} else if (position.equals(do_nothing)) {
					bot.run(new RobotInstruction()); // Do nothing after crossing
				} else {
					System.err.println("Unknown Posistion Auto Mode");
					bot.run(new RobotInstruction()); // Do nothing
				}
			} else if (ready_to_shoot) {
				bot.run(shoot_sm.process(bot.getCameraState())); // Shoot then stop (should proceed to wait to shoot state after shooting)
			} else {
				bot.run(new RobotInstruction()); // Do nothing (if finished with crossing and position and done shooting)
			}
		} else {
			System.err.println("Unknown Auto Mode");
			bot.run(new RobotInstruction()); // Do nothing
		}
	}
}