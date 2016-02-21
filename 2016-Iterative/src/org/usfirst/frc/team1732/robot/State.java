package org.usfirst.frc.team1732.robot;

public class State {
	private End finish;
	private Act act;
	
	public State(End finish, Act act) {
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

@FunctionalInterface
interface End {
	public boolean run(RobotState rbs);
}

@FunctionalInterface
interface Act {
	public RobotInstruction run(RobotState rbs);
}