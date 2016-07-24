package org.usfirst.frc.team1732.statemachine;

import java.util.ArrayList;
import org.usfirst.frc.team1732.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StateMachine<T> {
	
	private boolean isAuto = true;
	
	private ArrayList<State<T>> states = new ArrayList<State<T>>();
	private T current_state;
	private long start = System.currentTimeMillis();
	
	public void setState(T state) {
		current_state = state;
	}
	
	public T getState() {
		return current_state;
	}
	
	public StateMachine<T> addState(T state, End<T> finish) {
		states.add(new State<T>(state, ()->{}, finish));
		return this;
	}
	
	public StateMachine<T> addState(T state, Act act, End<T> finish) {
		states.add(new State<T>(state, act, finish));
		return this;
	}
	
	public void process() {
		Robot.bot.robotState.start_time = start;
		
		int state_index = -1;
		
		for (int i = 0; i < states.size(); i++) {
			if (states.get(i) == current_state) {
				state_index = i;
				break;
			}
		}
		
		T next = states.get(state_index).process();
		
		if (!next.equals(current_state)) {
			current_state = next;
			start = System.currentTimeMillis();
		}
		
		if(!isAuto) SmartDashboard.putString("State", current_state.toString());
		else SmartDashboard.putString("Auto State", current_state.toString());
	}
	
	public void setAuto(boolean b) {
		isAuto = b;
	}
}
