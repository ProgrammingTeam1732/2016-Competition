package org.usfirst.frc.team1732.statemachine;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StateMachine {
	private ArrayList<State> states = new ArrayList<State>();
	private int current_state;
	private long start = System.currentTimeMillis();
	
	public void reset() {
		current_state = 0;
	}
	
	public StateMachine addState(State in) {
		states.add(in);
		return this;
	}
	
	public RobotInstruction process(RobotState rbs) {
		rbs.start_time = start;
		RobotInstruction out = states.get(current_state).process(rbs);
		if (out.finished) {
			current_state++;
			start = System.currentTimeMillis();
			SmartDashboard.putNumber("Start Time", start);
			if (current_state >= states.size()) {
				current_state = 0;
			}
		}
		return out;
	}
}
