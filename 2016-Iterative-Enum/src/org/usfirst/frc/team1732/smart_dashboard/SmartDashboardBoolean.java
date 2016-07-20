package org.usfirst.frc.team1732.smart_dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardBoolean {
	private final String key;
	private boolean defaultValue;
	
	public SmartDashboardBoolean(String key, boolean defaultValue) {
		this.key = key;
		this.defaultValue = defaultValue;
		SmartDashboard.putBoolean(key, defaultValue);
	}
	
	public boolean get() {
		return SmartDashboard.getBoolean(key, defaultValue);
	}
	
	public void set(boolean value) {
		SmartDashboard.putBoolean(key, value);
	}
}