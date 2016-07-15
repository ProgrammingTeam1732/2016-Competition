package org.usfirst.frc.team1732.statemachine;

public class State<T> {
	private End<T> finish;
	private Act<T> act;
	private T name;
	
	public T getName() {
		return name;
	}
	
	public State(T name, Act<T> act, End<T> finish) {
		this.name = name;
		this.finish = finish;
		this.act = act;
	}
	
	public RobotInstruction<T> process(RobotState robot_state) {
		RobotInstruction<T> output = act.run(robot_state);
		output.next = finish.run(robot_state);
		if (output.next == null) output.next = name;
		return output;
	}
}