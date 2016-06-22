package org.usfirst.frc.team1732.statemachine;

@FunctionalInterface
public interface Act<T> {
	public RobotInstruction<T> run(RobotState rbs);
}