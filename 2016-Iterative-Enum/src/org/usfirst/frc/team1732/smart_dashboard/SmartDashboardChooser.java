package org.usfirst.frc.team1732.smart_dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardChooser <T>{
	
	/*	Example usage:
	 *	SmartDashboardChooser start_chooser = new SmartDashboardChooser("Start Mode", "Default Auto", default_auto_sm);
	 *	start_chooser.addKeys("Cross Defenses", "Approach Defenses");
	 *	start_chooser.addObjects(cross_terrain_sm, approach_defenses_sm);
	 *	start_chooser.display();
	 */
	
	SendableChooser chooser = new SendableChooser();
	String title = "";
	
	public SmartDashboardChooser(String title, String defaultKey, T defaultObject) {
		chooser.addDefault(defaultKey, defaultObject);
		this.title = title;
	}
	
	public void addObject(String key, T object) {
		chooser.addObject(key, object);
	}
	
	public void display() {
		SmartDashboard.putData(title, chooser);
	}
	
	@SuppressWarnings("unchecked")
	public T getSelected() {
		return (T) chooser.getSelected();
	}
}