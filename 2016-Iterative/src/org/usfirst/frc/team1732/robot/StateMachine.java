package org.usfirst.frc.team1732.robot;

import java.util.ArrayList;

public class StateMachine {
	private ArrayList<State> states = new ArrayList<State>();
	private int current_state;
	
	public StateMachine addState(State in) {
		states.add(in);
		return this;
	}
	
	public RobotInstruction process(RobotState rbs) {
		RobotInstruction out = states.get(current_state).process(rbs);
		if (out.finished) {
			current_state++;
			if (current_state >= states.size()) {
				current_state = 0;
			}
		}
		return out;
	}
}
