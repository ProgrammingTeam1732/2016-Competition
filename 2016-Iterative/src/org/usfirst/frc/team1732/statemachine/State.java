package org.usfirst.frc.team1732.statemachine;

public class State {
	private End finish;
	private Act act;
	
	public State(Act act, End finish) {
		this.finish = finish;
		this.act = act;
	}
	
	public RobotInstruction process(RobotState robot_state) {
		RobotInstruction output = act.run(robot_state);
		if (finish.run(robot_state)) {
			output.finished = true;
		}
		return output;
	}
}