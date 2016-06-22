package org.usfirst.frc.team1732.statemachine;

@FunctionalInterface
public interface End<T> {
	public T run(RobotState rbs);
}