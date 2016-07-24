package org.usfirst.frc.team1732.statemachine;

public class State<T> {
	private End<T> finish;
	private Act act;
	private T name;
	
	public T getName() {
		return name;
	}
	
	public State(T name, Act act, End<T> finish) {
		this.name = name;
		this.finish = finish;
		this.act = act;
	}
	
	public T process() {
		act.run();
		T next = finish.run();
		if (next == null) next = name;
		return next;
	}
}