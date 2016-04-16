
package org.usfirst.frc.team1732.robot;

import org.usfirst.frc.team1732.io.Input;
import org.usfirst.frc.team1732.statemachine.RobotInstruction;
import org.usfirst.frc.team1732.statemachine.RobotState;
import org.usfirst.frc.team1732.statemachine.State;
import org.usfirst.frc.team1732.statemachine.StateMachine;
import org.usfirst.frc.team1732.subsystems.Systems;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Relay;
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

	StateMachine pos2_sm = new StateMachine();
	StateMachine pos3_sm = new StateMachine();
	StateMachine pos4_sm = new StateMachine();
	StateMachine pos5_sm = new StateMachine();

	StateMachine approach = new StateMachine();

	StateMachine sm = new StateMachine();

	// double camera_angle = 0;
	// double camera_distance = 0;

	public void robotInit() {
		// bot.startCamera();

		try {
			CameraServer.getInstance().startAutomaticCapture("cam1");
			CameraServer.getInstance().setQuality(30);
		} catch (Exception e) {
			System.err.println(e);
			e.printStackTrace();
		}

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

		after_chooser.addDefault(default_auto, default_auto); // stop
		after_chooser.addObject(shoot, shoot); // shoot
		after_chooser.addObject(accelerate, accelerate); // approach
		SmartDashboard.putData("After Mode", after_chooser);

		pos_chooser.addDefault(pos1, pos1);
		pos_chooser.addObject(pos2, pos2);
		pos_chooser.addObject(pos3, pos3);
		pos_chooser.addObject(pos4, pos4);
		pos_chooser.addObject(pos5, pos5);
		pos_chooser.addObject(dont_shoot, dont_shoot);
		SmartDashboard.putData("Position", pos_chooser);

		sm.setAuto(false);
		sm.addState(new State("Wait to Shoot", (RobotState rbs) -> {
			return new RobotInstruction();
		} , (RobotState rbs) -> {
			if (rbs.shoot && rbs.fingers_open)
				return "Shoot Position";
			/*
			 * else if (rbs.shoot) return "Open Fingers";
			 */
			else
				return null;
		})).addState(new State("Shoot Position", (RobotState) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.catapult_in = true;
			return rbi;
		} , (RobotState rbs) -> {
			if (rbs.catapult_aligned_in && rbs.arm_aligned_high && rbs.fingers_open
					&& ((Math.abs(System.currentTimeMillis() - rbs.start_time) > 500)))
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

		rough_sm.setState("Arm Down");
		rough_sm.addState(new State("Arm Down", (RobotState rbs) -> {
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
			rbi.drive_left = 0.8 - (0.8 * ((System.currentTimeMillis() - rbs.start_time) / 2000.0));
			rbi.drive_right = 0.8 - (0.8 * ((System.currentTimeMillis() - rbs.start_time) / 2000.0));
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 2000)
				return "Finished";
			else
				return null;
		})).addState(new State("Finished", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			return rbi;
		} , (RobotState rbs) -> {
			return null;
		}));

		approach.setState("Accelerate");
		approach.addState(new State("Accelerate", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.drive_left = 0.5 * ((System.currentTimeMillis() - rbs.start_time) / 1000.0);
			rbi.drive_right = 0.5 * ((System.currentTimeMillis() - rbs.start_time) / 1000.0);
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 1000)
				return "Stop";
			else
				return null;
		})).addState(new State("Stop", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			return rbi;
		} , (RobotState rbs) -> {
			return null;
		}));

		/*
		 * .addState(new State("Stop", (RobotState rbs) -> { RobotInstruction
		 * rbi = new RobotInstruction(); return rbi; } , (RobotState rbs) -> {
		 * return null; }))
		 */

		low_bar_sm.setState("Drop Intake");
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
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 280) {
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
				return "Stop";
			}
			return null;
		})).addState(new State("Stop", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.arm_high = true;
			return rbi;
		} , (RobotState rbs) -> {
			return null;
		}));

		pos5_sm.setState("Center");
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
				return "Stop";
			}
			return null;
		})).addState(new State("Stop", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.arm_high = true;
			return rbi;
		} , (RobotState rbs) -> {
			return null;
		}));

		pos2_sm.setState("Center");
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
			if (((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0) * 0.095 > 53) {
				return "Stop";
			}
			return null;
		})).addState(new State("Stop", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.arm_high = true;
			return rbi;
		} , (RobotState rbs) -> {
			return null;
		}));

		// CAN ALSO DO SAME METHOD TO APPROACH BATTER PERPINDICULAR TO GOAL FOR
		// LOW BAR/POS 1
		// TODO Rewrite?
		/*
		 * pos2_sm.setState("Center"); pos2_sm.addState(new State("Center",
		 * (RobotState rbs) -> { RobotInstruction rbi = new RobotInstruction();
		 * if (rbs.gyro > 0) { rbi.drive_left = -0.4; rbi.drive_right = 0.4; }
		 * else { rbi.drive_left = 0.4; rbi.drive_right = -0.4; } return rbi; }
		 * , (RobotState rbs) -> { if (Math.abs(rbs.gyro) < 1) return
		 * "Point at Goal"; return null; })).addState(new State("Point at Goal",
		 * (RobotState rbs) -> { RobotInstruction rbi = new RobotInstruction();
		 * if (rbs.camera_angle == Double.NaN) { rbi.drive_left = 0.4;
		 * rbi.drive_right = -0.4; } else { rbi.drive_left =
		 * -limit(rbs.camera_angle / 6); rbi.drive_right =
		 * limit(rbs.camera_angle / 6); } return rbi; } , (RobotState rbs) -> {
		 * if (Math.abs(rbs.camera_angle) < 1) { camera_angle =
		 * Math.abs(rbs.gyro); camera_distance = rbs.distance_to_goal; return
		 * "Turn Back to Zero"; } else return null; })).addState(new State(
		 * "Turn Back to Zero", (RobotState rbs) -> { RobotInstruction rbi = new
		 * RobotInstruction(); if (rbs.gyro > 0) { rbi.drive_left = -0.4;
		 * rbi.drive_right = 0.4; } else { rbi.drive_left = 0.4; rbi.drive_right
		 * = -0.4; } return rbi; } , (RobotState rbs) -> { if
		 * (Math.abs(rbs.gyro) < 1) { bot.resetDriveEncoders(); return
		 * "Move One"; } return null; })).addState(new State("Move One",
		 * (RobotState rbs) -> { RobotInstruction rbi = new RobotInstruction();
		 * rbi.drive_left = 0.5; rbi.drive_right = 0.5; return rbi; } ,
		 * (RobotState rbs) -> { // If the robot has driven more than distance
		 * (see picture) if ((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0 *
		 * 0.095 > camera_distance Math.sin(Math.toRadians(180 - 120 -
		 * Math.abs(camera_angle)))) { return "Turn Two"; } return null;
		 * })).addState(new State("Turn Two", (RobotState rbs) -> {
		 * RobotInstruction rbi = new RobotInstruction(); rbi.drive_left = 0.4;
		 * rbi.drive_right = -0.4; return rbi; } , (RobotState rbs) -> { // Once
		 * robot has turned 60 degrees if (Math.abs(rbs.gyro) > 60) {
		 * bot.resetDriveEncoders(); return "Move Two"; } return null;
		 * })).addState(new State("Move Two", (RobotState rbs) -> {
		 * RobotInstruction rbi = new RobotInstruction(); rbi.drive_left = 0.4;
		 * rbi.drive_right = 0.4; return rbi; } , (RobotState rbs) -> { // If
		 * the robot has driven more than distance (see picture) 7.5 // inches
		 * is wheel diamater if (Math.abs(System.currentTimeMillis() -
		 * rbs.start_time) > 1000) { // if
		 * ((rbs.drive_left_dist+rbs.drive_left_dist)/2.0 * 0.095 > //
		 * rbs.distance / Math.sin(Math.toRadians(120)) * //
		 * Math.sin(Math.toRadians(Math.abs(rbs.turn_angle)))) { return "Stop";
		 * } return null; })).addState(new State("Stop", (RobotState rbs) -> {
		 * RobotInstruction rbi = new RobotInstruction(); return rbi; } ,
		 * (RobotState rbs) -> { return null; }));
		 */

		// TODO Rewrite?
		pos4_sm.setState("Turn Left");
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
				return "Stop";
			}
			return null;
		})).addState(new State("Stop", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.arm_high = true;
			return rbi;
		} , (RobotState rbs) -> {
			return null;
		}));

		// TODO Rewrite?
		pos3_sm.setState("Turn Right");
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
				return "Stop";
			}
			return null;
		})).addState(new State("Stop", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.arm_high = true;
			return rbi;
		} , (RobotState rbs) -> {
			return null;
		}));

		/*
		 * TODO Rewrite? pos5_sm.setState("Center"); pos5_sm.addState(new
		 * State("Center", (RobotState rbs) -> { RobotInstruction rbi = new
		 * RobotInstruction(); if (rbs.gyro > 0) { rbi.drive_left = -0.4;
		 * rbi.drive_right = 0.4; } else { rbi.drive_left = 0.4; rbi.drive_right
		 * = -0.4; } return rbi; } , (RobotState rbs) -> { if
		 * (Math.abs(rbs.gyro) < 1) return "Point at Goal"; return null;
		 * })).addState(new State("Point at Goal", (RobotState rbs) -> {
		 * RobotInstruction rbi = new RobotInstruction(); if (rbs.camera_angle
		 * == Double.NaN) { rbi.drive_left = -0.4; rbi.drive_right = 0.4; } else
		 * { rbi.drive_left = limit(rbs.camera_angle / 6); rbi.drive_right =
		 * -limit(rbs.camera_angle / 6); } return rbi; } , (RobotState rbs) -> {
		 * if (Math.abs(rbs.camera_angle) < 1) { camera_angle =
		 * Math.abs(rbs.gyro); camera_distance = rbs.distance_to_goal; return
		 * "Turn Back to Zero"; } else return null; })).addState(new State(
		 * "Turn Back to Zero", (RobotState rbs) -> { RobotInstruction rbi = new
		 * RobotInstruction(); if (rbs.gyro > 0) { rbi.drive_left = -0.4;
		 * rbi.drive_right = 0.4; } else { rbi.drive_left = 0.4; rbi.drive_right
		 * = -0.4; } return rbi; } , (RobotState rbs) -> { if
		 * (Math.abs(rbs.gyro) < 1) { bot.resetDriveEncoders(); return
		 * "Move One"; } return null; })).addState(new State("Move One",
		 * (RobotState rbs) -> { RobotInstruction rbi = new RobotInstruction();
		 * rbi.drive_left = 0.5; rbi.drive_right = 0.5; return rbi; } ,
		 * (RobotState rbs) -> { // If the robot has driven more than distance
		 * (see picture) if ((rbs.drive_left_dist + rbs.drive_left_dist) / 2.0 *
		 * 0.095 > camera_distance Math.sin(Math.toRadians(180 - 120 -
		 * Math.abs(camera_angle)))) { return "Turn Two"; } return null;
		 * })).addState(new State("Turn Two", (RobotState rbs) -> {
		 * RobotInstruction rbi = new RobotInstruction(); rbi.drive_left = -0.4;
		 * rbi.drive_right = 0.4; return rbi; } , (RobotState rbs) -> { // Once
		 * robot has turned 60 degrees if (Math.abs(rbs.gyro) > 60) {
		 * bot.resetDriveEncoders(); return "Move Two"; } return null;
		 * })).addState(new State("Move Two", (RobotState rbs) -> {
		 * RobotInstruction rbi = new RobotInstruction(); rbi.drive_left = 0.4;
		 * rbi.drive_right = 0.4; return rbi; } , (RobotState rbs) -> { // If
		 * the robot has driven more than distance (see picture) 7.5 // inches
		 * is wheel diamater if (Math.abs(System.currentTimeMillis() -
		 * rbs.start_time) > 1000) { // if
		 * ((rbs.drive_left_dist+rbs.drive_left_dist)/2.0 * 0.095 > //
		 * rbs.distance / Math.sin(Math.toRadians(120)) * //
		 * Math.sin(Math.toRadians(Math.abs(rbs.turn_angle)))) { return "Stop";
		 * } return null; })).addState(new State("Stop", (RobotState rbs) -> {
		 * RobotInstruction rbi = new RobotInstruction(); return rbi; } ,
		 * (RobotState rbs) -> { return null; }));
		 */

		// TODO Rewrite?
		portcullis_sm.setState("Accelerate");
		portcullis_sm.addState(new State("Accelerate", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.drive_left = 0.5 * ((System.currentTimeMillis() - rbs.start_time) / 1000.0);
			rbi.drive_right = 0.5 * ((System.currentTimeMillis() - rbs.start_time) / 1000.0);
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 1000)
				return "Stop";
			else
				return null;
		})).addState(new State("Stop", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			return rbi;
		} , (RobotState rbs) -> {
			return null;
		}));

		sally_port_sm.setState("Accelerate");
		sally_port_sm.addState(new State("Accelerate", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.drive_left = 0.5 * ((System.currentTimeMillis() - rbs.start_time) / 1000.0);
			rbi.drive_right = 0.5 * ((System.currentTimeMillis() - rbs.start_time) / 1000.0);
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 1000)
				return "Stop";
			else
				return null;
		})).addState(new State("Stop", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			return rbi;
		} , (RobotState rbs) -> {
			return null;
		}));

		drawbridge_sm.setState("Accelerate");
		drawbridge_sm.addState(new State("Accelerate", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.drive_left = 0.5 * ((System.currentTimeMillis() - rbs.start_time) / 1000.0);
			rbi.drive_right = 0.5 * ((System.currentTimeMillis() - rbs.start_time) / 1000.0);
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 1000)
				return "Stop";
			else
				return null;
		})).addState(new State("Stop", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			return rbi;
		} , (RobotState rbs) -> {
			return null;
		}));

		cheval_sm.setState("Accelerate");
		cheval_sm.addState(new State("Accelerate", (RobotState rbs) -> {
			RobotInstruction rbi = new RobotInstruction();
			rbi.drive_left = 0.5 * ((System.currentTimeMillis() - rbs.start_time) / 1000.0);
			rbi.drive_right = 0.5 * ((System.currentTimeMillis() - rbs.start_time) / 1000.0);
			return rbi;
		} , (RobotState rbs) -> {
			if (Math.abs(System.currentTimeMillis() - rbs.start_time) > 1000)
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

	public double limit(double in) {
		if (in > 0.5)
			return 0.5;
		if (in < -0.5)
			return -0.5;
		else
			return in;
	}

	long last;

	public void teleopPeriodic() {
		bot.run(sm.process(bot.getState(input.getShoot())), input);
		SmartDashboard.putNumber("TimeASUKDFYVA", System.currentTimeMillis() - last);
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
	final String accelerate = "Approach";
	String after;
	SendableChooser after_chooser = new SendableChooser();

	final String pos1 = "1";
	final String pos2 = "2";
	final String pos3 = "3";
	final String pos4 = "4";
	final String pos5 = "5";
	final String dont_shoot = "DOWNT SHEWT, DEWD";
	String pos;
	SendableChooser pos_chooser = new SendableChooser();

	boolean over = false;

	public void autonomousInit() {
		over = false;
		auto_mode = (String) chooser.getSelected();
		pos = (String) pos_chooser.getSelected();
		after = (String) after_chooser.getSelected();
		rough_sm.setState("Arm Down");
		pos2_sm.setState("Center");
		pos3_sm.setState("Center");
		pos4_sm.setState("Turn Left");
		pos5_sm.setState("Turn Right");
		bot.resetDriveEncoders();
		bot.resetGyro();
	}

	public void autonomousPeriodic() {
		if (after.equals(accelerate)) {
			bot.run(approach.process(bot.getState())); // run forward
		} else if (after.equals(default_auto)) {
			bot.run(sm.process(bot.getState())); // cock if not, moit?
		} else if (after.equals(shoot)) {
			if (!over) {
				if (auto_mode.equals(default_auto)) {
					bot.run(sm.process(bot.getState()));
				} else if (auto_mode.equals(rough)) {
					bot.run(rough_sm.process(bot.getState()));
					if (rough_sm.getState().equals("Finished"))
						over = true;
				} else if (auto_mode.equals(rock_wall)) {
					bot.run(rock_wall_sm.process(bot.getState()));
					if (rock_wall_sm.getState().equals("Finished"))
						over = true;
				} else if (auto_mode.equals(moat)) {
					bot.run(moat_sm.process(bot.getState()));
					if (moat_sm.getState().equals("Finished"))
						over = true;
				} else if (auto_mode.equals(cheval)) {
					bot.run(cheval_sm.process(bot.getState()));
					if (cheval_sm.getState().equals("Finished"))
						over = true;
				} else if (auto_mode.equals(low_bar)) {
					bot.run(low_bar_sm.process(bot.getState()));
					// does not progress to different auto
				} else if (auto_mode.equals(drawbridge)) {
					bot.run(drawbridge_sm.process(bot.getState()));
					if (drawbridge_sm.getState().equals("Finished"))
						over = true;
				} else if (auto_mode.equals(portcullis)) {
					bot.run(portcullis_sm.process(bot.getState()));
					if (portcullis_sm.getState().equals("Finished"))
						over = true;
				} else if (auto_mode.equals(ramparts)) {
					bot.run(ramparts_sm.process(bot.getState()));
					if (ramparts_sm.getState().equals("Finished"))
						over = true;

				} else if (auto_mode.equals(sally_port)) {
					bot.run(sally_port_sm.process(bot.getState()));
					if (sally_port_sm.getState().equals("Finished"))
						over = true;
				} else {
					System.err.println("Unknown Crossing Auto Mode");
					bot.run(sm.process(bot.getState()));
				}
			} else { // we are over the obstacle(?)
				if (pos.equals(pos2)) {
					if (pos2_sm.getState() == "Camera") {
						bot.run(pos2_sm.process(bot.getCameraState()));
					} else {
						bot.run(pos2_sm.process(bot.getState()));
					}
				} else if (pos.equals(pos3)) {
					bot.run(pos3_sm.process(bot.getState()));
				} else if (pos.equals(pos4)) {
					bot.run(pos4_sm.process(bot.getState()));
				} else if (pos.equals(pos5)) {
					bot.run(pos5_sm.process(bot.getState()));
				} else if (pos.equals(dont_shoot)) {
					bot.run(sm.process(bot.getState())); // cock if not, moit?
				} else {
					System.err.println("Unknown Shooting Auto Mode");
					bot.run(sm.process(bot.getState()));
				}
			}
		} else {
			System.err.println("Unknown Auton Mode");
			bot.run(sm.process(bot.getState()));
		}
	}
}