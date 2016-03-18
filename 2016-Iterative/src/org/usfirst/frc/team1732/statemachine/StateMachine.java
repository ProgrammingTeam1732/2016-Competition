package org.usfirst.frc.team1732.statemachine;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StateMachine {
	private ArrayList<State> states = new ArrayList<State>();
	private String current_state = "Wait to Shoot";
	private long start = System.currentTimeMillis();
	
	public void setState(String state) {
		current_state = state;
	}
	
	public String getState() {
		return current_state;
	}
	
	public StateMachine addState(State in) {
		states.add(in);
		return this;
	}
	
	public RobotInstruction process(RobotState rbs) {
		rbs.start_time = start;
		
		int state_index = -1;
		
		for (int i = 0; i < states.size(); i++) {
			if (states.get(i).getName() == current_state) {
				state_index = i;
				break;
			}
		}
		
		if (state_index == -1) {
			SmartDashboard.putString("State", "State not found: " + current_state);
			return new RobotInstruction();
		}
		
		RobotInstruction out = states.get(state_index).process(rbs);
		
		if (!out.next.equals(current_state)) {
			current_state = out.next;
			
			start = System.currentTimeMillis();
		}
		
		SmartDashboard.putString("State", current_state);
		return out;
	}
}
