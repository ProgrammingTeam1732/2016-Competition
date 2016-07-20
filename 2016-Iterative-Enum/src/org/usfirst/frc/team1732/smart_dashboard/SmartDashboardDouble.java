package org.usfirst.frc.team1732.smart_dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardDouble {
	private final String key;
	private double defaultValue;

	public SmartDashboardDouble(String key, double defaultValue) {
		this.key = key;
		this.defaultValue = defaultValue;
		SmartDashboard.putNumber(key, defaultValue);
	}

	public double get() {
		return SmartDashboard.getNumber(key, defaultValue);
	}
	
	public void set(double value) {
		SmartDashboard.putNumber(key, value);
	}
}