package org.usfirst.frc.team1732.statemachine;

public class State {
	private End finish;
	private Act act;
	private String name;
	
	public String getName() {
		return name;
	}
	
	public State(String name, Act act, End finish) {
		this.name = name;
		this.finish = finish;
		this.act = act;
	}
	
	public RobotInstruction process(RobotState robot_state) {
		RobotInstruction output = act.run(robot_state);
		output.next = finish.run(robot_state);
		if (output.next == null) output.next = name;
		return output;
	}
}