
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

	StateMachine cross_terrain = new StateMachine();
	StateMachine portcullis_sm = new StateMachine();
	StateMachine cheval_sm = new StateMachine();
	StateMachine drawbridge_sm = new StateMachine();
	StateMachine sally_port_sm = new StateMachine();
	StateMachine low_bar_sm = new StateMachine();

	StateMachine pos2_sm = new StateMachine();
	StateMachine pos3_sm = new StateMachine();
	StateMachine pos4_sm = new StateMachine();
	StateMachine pos5_sm = new StateMachine();

	StateMachine approach = new StateMachine();

	StateMachine sm = new StateMachine();

	public void robotInit() {
		bot = new Systems();
		start_chooser.addDefault(default_auto, default_auto); // stop
		start_chooser.addObject(cross_defenses, cross_defenses); // cross
																	// defenses
		start_chooser.addObject(approach_defenses, approach_defenses); // approach
																		// defenses
		SmartDashboard.putData("Start Mode", start_chooser);

		cross_chooser.addDefault(default_auto, default_auto);
		cross_chooser.addObject(cross_terrain_auto, cross_terrain_auto);
		cross_chooser.addObject(portcullis_auto, portcullis_auto);
		cross_chooser.addObject(low_bar_auto, low_bar_auto);
		cross_chooser.addObject(cheval_auto, cheval_auto);
		cross_chooser.addObject(sally_port_auto, sally_port_auto);
		cross_chooser.addObject(drawbridge_auto, drawbridge_auto);
		SmartDashboard.putData("Crossing Mode", cross_chooser);

		pos_chooser.addDefault(pos1, pos1);
		pos_chooser.addObject(pos2, pos2);
		pos_chooser.addObject(pos3, pos3);
		pos_chooser.addObject(pos4, pos4);
		pos_chooser.addObject(pos5, pos5);
		pos_chooser.addObject(dont_shoot, dont_shoot);
		SmartDashboard.putData("Position", pos_chooser);
		// Generic add state:
		/*
		 * .addState(new State("", (RobotState rbs) -> { RobotInstruction rbi =
		 * new RobotInstruction(); return rbi; }, (RobotState rbs) -> { if(true)
		 * return ""; else return null; }))
		 * 
		 */
		// TODO: as
		// SM Here asdad
		sm.addState(new State("Wait to Shoot", (RobotState rbs) -> {
			RobotInstruction rbi =  new RobotInstruction();
			return rbi;
		} , (RobotState rbs) -> {
			if (rbs.shoot && rbs.fingers_open)
				return "Shoot Mode";
			if (rbs.reset_catapult && rbs.fingers_open)
				return "Shoot Mode";
			/*
			 * else if (rbs.shoot) return "Open Fingers";
			 */
			else
				return null;
		})).addState(new State("Auto Init", (RobotState rbs) -> {
			return new RobotInstruction();
		} , (RobotState rbs) -> {
			if (rbs.arm_aligned_high)
				return "Shoot Position";
			/*
			 * else if (rbs.shoot) return "Open Fingers";
			 */
			else
				return null;
		})).addState(new State("Shoot Mode", (RobotState rbs) -> {
			return new RobotInstruction();
		} , (RobotState rbs) -> {
			if (rbs.shoot_mode_far || rbs.shoot_mode_close)
				return "Shoot Position";
			else if (!rbs.camera_exists)
				return "Wait to Shoot";
			else if (rbs.shoot_mode_auto)
				return "Point at Goal";
			else
				return null;
		})).addState(new State("Point at Goal", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
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
		} , (RobotState rbs) -> {
			// FIXME: change this after testing turning
			// FIXME
			if (Math.abs(0.5 - rbs.angle_to_goal) < 0.1)
				return "Wait to Shoot";
			else if (!rbs.shoot_mode_auto)
				return "Wait to Shoot";
			else
				return null;
		})).addState(new State("Auto Shoot Posistion", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			// TODO: calculate function for setpoint based on distance
			// FIXME
			rbi.catapult_auto_pos = (int) (rbs.distance_to_goal * 2);
			rbi.catapult_shoot = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (rbs.catapult_aligned_shoot && rbs.arm_aligned_high && rbs.fingers_open
					&& ((Math.abs(System.currentTimeMillis() - rbs.start_time) > 500)))
				return "Shoot";
			else
				return null;
		})).addState(new State("Shoot Position", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.catapult_shoot = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (rbs.catapult_aligned_shoot && rbs.arm_aligned_high && rbs.fingers_open
					&& ((Math.abs(System.currentTimeMillis() - rbs.start_time) > 500)))
				return "Shoot";
			if (rbs.reset_catapult && rbs.fingers_open)
				return "Shoot";
			else
				return null;
		}))/*
			 * .addState(new State("Open Fingers", (RobotState rbs) -> {
			 * RobotInstruction rbi = new RobotInstruction(); //
			 * rbi.fingers_open = true; return rbi; } , (RobotState rbs) -> {
			 * //if ((Math.abs(System.currentTimeMillis() - rbs.start_time) >
			 * 200)) return "Shoot Position"; //else // return null; }))
			 */.addState(new State("Shoot", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.catapult_release = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 200)
				return "Retrive Tram";
			else
				return null;
		})).addState(new State("Retrive Tram", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.catapult_out = true;
			//rbi.catapult_release = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (rbs.catapult_aligned_out)
				return "Latch Tram";
			else
				return null;
		})).addState(new State("Latch Tram", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.catapult_latch = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 500)
				return "Pull Tram";
			else
				return null;
		})).addState(new State("Pull Tram", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.catapult_load = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (rbs.catapult_aligned_load)
				return "Wait to Shoot";
			else
				return null;
		}));

		// SM Here
		cross_terrain.addState(new State("Arm Down", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.intake_down = true;
			rbi.arm_middle = true;
			return rbi;
		} , (RobotState rbs) -> { // FIXME?
			if (rbs.arm_aligned_middle)
				return "Drive Acc";
			else
				return null;
		})).addState(new State("Drive Acc", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.drive_left = 0.8 * ((System.currentTimeMillis() - rbs.start_time) / 2500.0);
			rbi.drive_right = 0.8 * ((System.currentTimeMillis() - rbs.start_time) / 2500.0);
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 2500)
				return "Drive Steady";
			else
				return null;
		})).addState(new State("Drive Steady", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.drive_left = 0.8 - (0.8 * ((System.currentTimeMillis() - rbs.start_time) / 3000.0));
			rbi.drive_right = 0.8 - (0.8 * ((System.currentTimeMillis() - rbs.start_time) / 3000.0));
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 3000)
				return "Finished";
			else
				return null;
		})).addState(new State("Finished", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			return rbi;
		} , (RobotState rbs) -> {
			return null;
		}));

		// SM Here
		approach.addState(new State("Accelerate", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.drive_left = 0.5 * ((System.currentTimeMillis() - rbs.start_time) / 1000.0);
			rbi.drive_right = 0.5 * ((System.currentTimeMillis() - rbs.start_time) / 1000.0);
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 1000)
				return "Finished";
			else
				return null;
		})).addState(new State("Finished", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			return rbi;
		} , (RobotState rbs) -> {
			return null;
		}));

		// SM Here
		/*
		 * double distance = 118; double left = 174; double right = 150; double
		 * ratio = right / left; double scalar = 0.5; double batter = 48;
		 * low_bar_sm.addState(new State("Drop Intake", (RobotState rbs) -> {
		 * RobotInstruction rbi = new RobotInstruction(); rbi.intake_down =
		 * true; return rbi; } , (RobotState rbs) -> { if
		 * (Math.abs(System.currentTimeMillis() - rbs.start_time) > 1000) return
		 * "Drop Arm"; else return null; })).addState(new State("Drop Arm",
		 * (RobotState rbs) -> { RobotInstruction rbi = new RobotInstruction();
		 * rbi.catapult_load = true; rbi.fingers_close = true; if
		 * (rbs.catapult_aligned_load && rbs.fingers_closed && rbs.intake_down)
		 * { rbi.arm_low = true; } return rbi; } , (RobotState rbs) -> { if
		 * (Math.abs(System.currentTimeMillis() - rbs.start_time) > 1000) return
		 * "Wait"; else return null; })).addState(new State("Wait", (RobotState
		 * rbs) -> { RobotInstruction rbi = new RobotInstruction(); return rbi;
		 * } , (RobotState rbs) -> { if (rbs.arm_aligned_low) return
		 * "Drive Forward"; else return null; })).addState(new State(
		 * "Drive Forward", (RobotState rbs) -> { RobotInstruction rbi = new
		 * RobotInstruction(); rbi.reset_gyro = true; rbi.drive_left = 0.345;
		 * rbi.drive_right = 0.345; return rbi; } , (RobotState rbs) -> { if
		 * (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 >
		 * distance) { return "Wait2"; } return null; })).addState(new
		 * State("Wait2", (RobotState rbs) -> { RobotInstruction rbi = new
		 * RobotInstruction(); rbi.reset_drive = true; return rbi; } ,
		 * (RobotState rbs) -> { if (System.currentTimeMillis() - rbs.start_time
		 * > 100) return "Drive Curve"; return null; })).addState(new State(
		 * "Drive Curve", (RobotState rbs) -> { RobotInstruction rbi = new
		 * RobotInstruction(); rbi.drive_left = 1 * scalar; rbi.drive_right =
		 * ratio * scalar; return rbi; } , (RobotState rbs) -> { if
		 * (rbs.drive_left_dist* 0.095 > left || rbs.drive_right_dist* 0.095 >
		 * right && rbs.intake_down) return "Raise Arm"; return null;
		 * })).addState(new State("Raise Arm", (RobotState rbs) -> {
		 * RobotInstruction rbi = new RobotInstruction(); rbi.arm_high = true;
		 * return rbi; } , (RobotState rbs) -> { if (rbs.arm_aligned_high)
		 * return "Open Fingers"; else return null; })).addState(new State(
		 * "Open Fingers", (RobotState rbs) -> { RobotInstruction rbi = new
		 * RobotInstruction(); rbi.fingers_open = true; return rbi; } ,
		 * (RobotState rbs) -> { if (System.currentTimeMillis() - rbs.start_time
		 * > 500) return "Finished"; else return null; })).addState(new
		 * State("Finished", (RobotState rbs) -> { return new
		 * RobotInstruction(); } , (RobotState rbs) -> { return null; }));
		 */

		low_bar_sm.addState(new State("Drop Intake", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.intake_down = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 1000)
				return "Drop Arm";
			else
				return null;
		})).addState(new State("Drop Arm", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.catapult_load = true;
			rbi.fingers_close = true;
			if (rbs.catapult_aligned_load && rbs.fingers_closed && rbs.intake_down) {
				rbi.arm_low = true;
			}
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 1000)
				return "Wait";
			else
				return null;
		})).addState(new State("Wait", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			return rbi;
		} , (RobotState rbs) -> {
			if (rbs.arm_aligned_low)
				return "Drive Forward";
			else
				return null;
		})).addState(new State("Drive Forward", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.reset_gyro = true;
			rbi.drive_left = 0.345;
			rbi.drive_right = 0.345;
			return rbi;
		} , (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 220) {
				return "Turn";
			}
			return null;
		})).addState(new State("Turn", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			if (rbs.gyro > 55) {
				rbi.drive_left = -0.4;
				rbi.drive_right = 0.4;
			} else {
				rbi.drive_left = 0.4;
				rbi.drive_right = -0.4;
			}
			rbi.reset_drive = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(rbs.gyro - 55) < 1) {
				return "Drive Forward Two";
			}
			return null;
		})).addState(new State("Drive Forward Two", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.drive_left = 0.2;
			rbi.drive_right = 0.2;
			return rbi;
		} , (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 160) {
				return "Raise Arm";
			}
			return null;
		})).addState(new State("Raise Arm", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.arm_high = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (System.currentTimeMillis() - rbs.start_time > 500)
				return "Open Fingers";
			else
				return null;
		})).addState(new State("Open Fingers", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.fingers_open = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (System.currentTimeMillis() - rbs.start_time > 500)
				return "Finished";
			else
				return null;
		})).addState(new State("Finished", (RobotState rbs) -> {
			return new RobotInstruction();
		} , (RobotState rbs) -> {
			return null;
		}));

		// SM Here
		pos5_sm.addState(new State("Center", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			if (rbs.gyro > 0) {
				rbi.drive_left = -0.4;
				rbi.drive_right = 0.4;
			} else {
				rbi.drive_left = 0.4;
				rbi.drive_right = -0.4;
			}
			rbi.reset_drive = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(rbs.gyro) < 1) {
				return "Drive Forward";
			}
			return null;
		})).addState(new State("Drive Forward", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			/*
			 * if (dist < 5) { drive_speed = 0.3; } else if (dist < 30) {
			 * drive_speed = 0.6 * (dist) / 30.0; } else if (dist > 90) {
			 * drive_speed = -1/30.0 * dist + (18/5.0); } else { drive_speed =
			 * 0.6; }
			 */

			rbi.drive_left = 0.345;
			rbi.drive_right = 0.345;
			rbi.reset_gyro = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 120) {
				return "Turn";
			}
			return null;
		})).addState(new State("Turn", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			if (rbs.gyro > -55) {
				rbi.drive_left = -0.4;
				rbi.drive_right = 0.4;
			} else {
				rbi.drive_left = 0.4;
				rbi.drive_right = -0.4;
			}
			rbi.reset_drive = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(rbs.gyro + 50) < 1) {
				return "Drive Forward Two";
			}
			return null;
		})).addState(new State("Drive Forward Two", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();

			/*
			 * if (dist < 5) { drive_speed = 0.3; } else if (dist < 30) {
			 * drive_speed = 0.6 * (dist) / 30.0; } else if (dist > 90) {
			 * drive_speed = -1/30.0 * dist + (18/5.0); } else { drive_speed =
			 * 0.6; }
			 */

			rbi.drive_left = 0.2;
			rbi.drive_right = 0.2;
			return rbi;
		} , (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 58) {
				return "Finished";
			}
			return null;
		})).addState(new State("Finished", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.arm_high = true;
			return rbi;
		} , (RobotState rbs) -> {
			return null;
		}));

		// SM Here
		pos2_sm.addState(new State("Center", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			if (rbs.gyro > 0) {
				rbi.drive_left = -0.4;
				rbi.drive_right = 0.4;
			} else {
				rbi.drive_left = 0.4;
				rbi.drive_right = -0.4;
			}
			rbi.reset_drive = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(rbs.gyro) < 1) {
				return "Drive Forward";
			}
			return null;
		})).addState(new State("Drive Forward", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			/*
			 * if (dist < 5) { drive_speed = 0.3; } else if (dist < 30) {
			 * drive_speed = 0.6 * (dist) / 30.0; } else if (dist > 90) {
			 * drive_speed = -1/30.0 * dist + (18/5.0); } else { drive_speed =
			 * 0.6; }
			 */
			rbi.reset_gyro = true;
			rbi.drive_left = 0.345;
			rbi.drive_right = 0.345;
			return rbi;
		} , (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 105) {
				return "Turn";
			}
			return null;
		})).addState(new State("Turn", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			if (rbs.gyro > +55) {
				rbi.drive_left = -0.4;
				rbi.drive_right = 0.4;
			} else {
				rbi.drive_left = 0.4;
				rbi.drive_right = -0.4;
			}
			rbi.reset_drive = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(rbs.gyro - 55) < 1) {
				return "Drive Forward Two";
			}
			return null;
		})).addState(new State("Drive Forward Two", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.drive_left = 0.2;
			rbi.drive_right = 0.2;
			return rbi;
		} , (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 53) {
				return "Finished";
			}
			return null;
		})).addState(new State("Finished", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.arm_high = true;
			return rbi;
		} , (RobotState rbs) -> {
			return null;
		}));

		// SM Here
		pos4_sm.addState(new State("Turn Left", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			if (rbs.gyro > -15) {
				rbi.drive_left = -0.4;
				rbi.drive_right = 0.4;
			} else {
				rbi.drive_left = 0.4;
				rbi.drive_right = -0.4;
			}
			rbi.reset_drive = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(rbs.gyro + 15) < 1) {
				return "Drive Forward";
			}
			return null;
		})).addState(new State("Drive Forward", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();

			/*
			 * if (dist < 5) { drive_speed = 0.3; } else if (dist < 30) {
			 * drive_speed = 0.6 * (dist) / 30.0; } else if (dist > 90) {
			 * drive_speed = -1/30.0 * dist + (18/5.0); } else { drive_speed =
			 * 0.6; }
			 */

			rbi.drive_left = 0.345;
			rbi.drive_right = 0.345;
			rbi.reset_gyro = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 62) {
				return "Turn Right";
			}
			return null;
		})).addState(new State("Turn Right", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			if (rbs.gyro > 15) {
				rbi.drive_left = -0.4;
				rbi.drive_right = 0.4;
			} else {
				rbi.drive_left = 0.4;
				rbi.drive_right = -0.4;
			}
			rbi.reset_drive = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(rbs.gyro - 15) < 1) {
				return "Drive Forward Two";
			}
			return null;
		})).addState(new State("Drive Forward Two", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			/*
			 * if (dist < 5) { drive_speed = 0.3; } else if (dist < 30) {
			 * drive_speed = 0.6 * (dist) / 30.0; } else if (dist > 90) {
			 * drive_speed = -1/30.0 * dist + (18/5.0); } else { drive_speed =
			 * 0.6; }
			 */

			rbi.drive_left = 0.2;
			rbi.drive_right = 0.2;
			return rbi;
		} , (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 89) {
				return "Finished";
			}
			return null;
		})).addState(new State("Finished", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.arm_high = true;
			return rbi;
		} , (RobotState rbs) -> {
			return null;
		}));

		// SM Here
		pos3_sm.addState(new State("Turn Right", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			if (rbs.gyro > 40) {
				rbi.drive_left = -0.4;
				rbi.drive_right = 0.4;
			} else {
				rbi.drive_left = 0.4;
				rbi.drive_right = -0.4;
			}
			rbi.reset_drive = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(rbs.gyro - 40) < 1) {
				return "Drive Forward";
			}
			return null;
		})).addState(new State("Drive Forward", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();

			/*
			 * if (dist < 5) { drive_speed = 0.3; } else if (dist < 30) {
			 * drive_speed = 0.6 * (dist) / 30.0; } else if (dist > 90) {
			 * drive_speed = -1/30.0 * dist + (18/5.0); } else { drive_speed =
			 * 0.6; }
			 */

			rbi.drive_left = 0.345;
			rbi.drive_right = 0.345;
			rbi.reset_gyro = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 54) {
				return "Turn Left";
			}
			return null;
		})).addState(new State("Turn Left", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			if (Math.abs(rbs.gyro) > -40) {
				rbi.drive_left = -0.4;
				rbi.drive_right = 0.4;
			} else {
				rbi.drive_left = 0.4;
				rbi.drive_right = -0.4;
			}
			rbi.reset_drive = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(rbs.gyro + 40) < 1) {
				return "Drive Forward Two";
			}
			return null;
		})).addState(new State("Drive Forward Two", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();

			/*
			 * if (dist < 5) { drive_speed = 0.3; } else if (dist < 30) {
			 * drive_speed = 0.6 * (dist) / 30.0; } else if (dist > 90) {
			 * drive_speed = -1/30.0 * dist + (18/5.0); } else { drive_speed =
			 * 0.6; }
			 */

			rbi.drive_left = 0.2;
			rbi.drive_right = 0.2;
			return rbi;
		} , (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 111) {
				return "Finished";
			}
			return null;
		})).addState(new State("Finished", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.arm_high = true;
			return rbi;
		} , (RobotState rbs) -> {
			return null;
		}));

		// SM Here
		portcullis_sm.addState(new State("Accelerate", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			// rbi.reset_defense = true; not needed if lowering defense at same
			// time
			rbi.intake_down = true;
			rbi.defense_down = true;
			rbi.arm_auto = true;
			rbi.drive_left = 0.345;
			rbi.drive_right = 0.345;
			return rbi;
		} , (RobotState rbs) -> {
			if (rbs.arm_aligned_auto && rbs.manip_encoder / 10.0 > 0.25) {
				// FIXME: counts per rotation
				return "Continue Drive";
			}
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 21) {
				return "Continue Arm/Defense";
			}
			return null;
		})).addState(new State("Continue Arm/Defense", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.defense_down = true;
			rbi.arm_auto = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (rbs.arm_aligned_auto && rbs.manip_encoder / 10.0 > 0.25) {
				// FIXME: counts per rotation
				return "Arm Low";
			}
			return null;
		})).addState(new State("Continue Drive", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.drive_left = 0.345;
			rbi.drive_right = 0.345;
			return rbi;
		} , (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 21) {
				return "Arm Low";
			}
			return null;
		})).addState(new State("Arm Low", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.arm_low = true;
			rbi.drive_left = .345;
			rbi.drive_right = .345;
			return rbi;
		} , (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 55) {
				return "Finished";
			} else
				return null;
		})).addState(new State("Finished", (RobotState rbs) -> {
			return new RobotInstruction();
		} , (RobotState rbs) -> {
			return null;
		}));

		// SM Here
		sally_port_sm.addState(new State("Accelerate", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.drive_left = 0.5 * ((System.currentTimeMillis() - rbs.start_time) / 1000.0);
			rbi.drive_right = 0.5 * ((System.currentTimeMillis() - rbs.start_time) / 1000.0);
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 1000)
				return "Finished";
			else
				return null;
		})).addState(new State("Finished", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			return rbi;
		} , (RobotState rbs) -> {
			return null;
		}));

		// SM Here
		drawbridge_sm.addState(new State("Accelerate", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.drive_left = 0.5 * ((System.currentTimeMillis() - rbs.start_time) / 1000.0);
			rbi.drive_right = 0.5 * ((System.currentTimeMillis() - rbs.start_time) / 1000.0);
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 1000)
				return "Finished";
			else
				return null;
		})).addState(new State("Finished", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			return rbi;
		} , (RobotState rbs) -> {
			return null;
		}));

		// SM Here
		cheval_sm.addState(new State("Accelerate", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			// rbi.drive_left = 0.5 * ((System.currentTimeMillis() -
			// rbs.start_time) / 1000.0);
			// rbi.drive_right = 0.5 * ((System.currentTimeMillis() -
			// rbs.start_time) / 1000.0);
			rbi.arm_auto = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (rbs.arm_aligned_auto)
				return "Drive";
			else
				return null;
		})).addState(new State("Drive", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.drive_left = .345;
			rbi.drive_right = .345;
			return rbi;
		} , (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 21) {
				return "Low Mans";
			} else
				return null;
		})).addState(new State("Low Mans", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.defense_down = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (rbs.manip_encoder / 10.0 > 0.25)
				return "Drive2";
			return null;
		})).addState(new State("Drive2", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.drive_left = .345;
			rbi.drive_right = .345;
			rbi.arm_middle = true;
			rbi.reset_defense = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 55) {
				return "Raise Mans";
			} else
				return null;
		})).addState(new State("Raise Mans", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.defense_up = true;
			return rbi;
		} , (RobotState rbs) -> {
			// FIXME: determine positive and negative direction on encoder for
			// mans
			if (rbs.manip_encoder / 10.0 < -0.25)
				return "Finished";
			return null;
		})).addState(new State("Finished", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			return rbi;
		} , (RobotState rbs) -> {
			return null;
		}));
	}

	public double limit(double in) {
		if (in > 0.5)
			return 0.5;
		if (in < -0.5)
			return -0.5;
		else
			return in;
	}

	private long last;

	public void teleopInit() {
		sm.setAuto(false);
	}

	public void teleopPeriodic() {
		if (sm.getState().equals("Auto Init")) sm.setState("Wait to Shoot");
		if (sm.getState().equals("Point at Goal"))
			bot.run(sm.process(bot.getCameraState()), input);
		else bot.run(sm.process(bot.getState(input.getShoot(), input.getResetShot())), input);
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

	long start_time;

	boolean once = true;

	final String default_auto = "Default Auto";
	final String cross_terrain_auto = "Cross Terrain Auto";
	final String portcullis_auto = "Portcullis Auto";
	final String cheval_auto = "Cheval Auto";
	final String drawbridge_auto = "Draw Bridge Auto";
	final String sally_port_auto = "Sally Port Auto";
	final String low_bar_auto = "Low Bar Auto";
	String cross_mode;
	SendableChooser cross_chooser = new SendableChooser();

	final String cross_defenses = "Cross Defenses";
	final String approach_defenses = "Approach Defenses";
	String start_mode;
	SendableChooser start_chooser = new SendableChooser();

	final String pos1 = "1";
	final String pos2 = "2";
	final String pos3 = "3";
	final String pos4 = "4";
	final String pos5 = "5";
	final String dont_shoot = "Don't Move After Crossing";
	String pos;
	SendableChooser pos_chooser = new SendableChooser();

	boolean over = false;
	boolean shoot = false;

	public void autonomousInit() {
		over = false;
		start_mode = (String) start_chooser.getSelected();
		cross_mode = (String) cross_chooser.getSelected();
		pos = (String) pos_chooser.getSelected();

		cross_terrain.setState("Arm Down");
		approach.setState("Accelerate");
		low_bar_sm.setState("Drop Intake");
		portcullis_sm.setState("Accelerate");
		sally_port_sm.setState("Accelerate");
		drawbridge_sm.setState("Accelerate");
		cheval_sm.setState("Accelerate");

		pos2_sm.setState("Center");
		pos3_sm.setState("Turn Right");
		pos4_sm.setState("Turn Left");
		pos5_sm.setState("Center");

		sm.setAuto(true);
		sm.setState("Auto Init");

		bot.prepareAuto();
	}

	// TODO:
	public void autonomousPeriodic() {
		if (start_mode.equals(default_auto)) {
			bot.run(new RobotInstruction()); // do nothing
		} else if (start_mode.equals(approach_defenses)) {
			bot.run(approach.process(bot.getState())); // approach batter
		} else if (start_mode.equals(cross_defenses)) {
			if (!over) {
				if (cross_mode.equals(default_auto)) {
					bot.run(sm.process(bot.getState()));
				} else if (cross_mode.equals(cross_terrain_auto)) {
					bot.run(cross_terrain.process(bot.getState()));
					if (cross_terrain.getState().equals("Finished"))
						over = true;
				} else if (cross_mode.equals(cheval_auto)) {
					bot.run(cheval_sm.process(bot.getState()));
					if (cheval_sm.getState().equals("Finished"))
						over = true;
				} else if (cross_mode.equals(low_bar_auto)) {
					bot.run(low_bar_sm.process(bot.getState()));
					if (low_bar_sm.equals("Finished"))
						over = true;
				} else if (cross_mode.equals(drawbridge_auto)) {
					bot.run(drawbridge_sm.process(bot.getState()));
					if (drawbridge_sm.getState().equals("Finished"))
						over = true;
				} else if (cross_mode.equals(portcullis_auto)) {
					bot.run(portcullis_sm.process(bot.getState()));
					if (portcullis_sm.getState().equals("Finished"))
						over = true;
				} else if (cross_mode.equals(sally_port_auto)) {
					bot.run(sally_port_sm.process(bot.getState()));
					if (sally_port_sm.getState().equals("Finished"))
						over = true;
				} else {
					System.err.println("Unknown Crossing Auto Mode");
					bot.run(new RobotInstruction()); // do nothing
				}
			} else if (!shoot) { // crossing state machine has finished, not
									// readt to shoot
				if (pos.equals(pos1)) {
					if (low_bar_sm.getState().equals("Finished"))
						shoot = false;
				} else if (pos.equals(pos2)) {
					bot.run(pos2_sm.process(bot.getState()));
					if (pos2_sm.getState().equals("Finished"))
						shoot = false;
				} else if (pos.equals(pos3)) {
					bot.run(pos3_sm.process(bot.getState()));
					if (pos3_sm.getState().equals("Finished"))
						shoot = false;
				} else if (pos.equals(pos4)) {
					bot.run(pos4_sm.process(bot.getState()));
					if (pos4_sm.getState().equals("Finished"))
						shoot = false;
				} else if (pos.equals(pos5)) {
					bot.run(pos5_sm.process(bot.getState()));
					if (pos5_sm.getState().equals("Finished"))
						shoot = false;
				} else if (pos.equals(dont_shoot)) {
					bot.run(new RobotInstruction()); // do nothing after
														// crossing
				} else {
					System.err.println("Unknown Posistion Auto Mode");
					bot.run(new RobotInstruction()); // do nothing
				}
			} else if (shoot) {
				bot.run(sm.process(bot.getState())); // shoot, will not do
														// anything after
														// shooting
			} else {
				bot.run(new RobotInstruction()); // do nothing (if finished with
													// crossing and position and
													// done shooting)
			}
		} else {
			System.err.println("Unknown Auto Mode");
			bot.run(new RobotInstruction());
		}
	}
}