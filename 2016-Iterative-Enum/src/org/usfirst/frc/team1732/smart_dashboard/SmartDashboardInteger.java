package org.usfirst.frc.team1732.smart_dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardInteger {
	private final String key;
	private int defaultValue;
	
	public SmartDashboardInteger(String key, int defaultValue) {
		this.key = key;
		this.defaultValue = defaultValue;
		SmartDashboard.putNumber(key, defaultValue);
	}
	
	public int get() {
		return (int)SmartDashboard.getNumber(key, defaultValue);
	}
	
	public void set(int value) {
		SmartDashboard.putNumber(key, value);
	}
}