package org.usfirst.frc.team1732.statemachine;

@FunctionalInterface
public interface Act {
	public RobotInstruction run(RobotState rbs);
}