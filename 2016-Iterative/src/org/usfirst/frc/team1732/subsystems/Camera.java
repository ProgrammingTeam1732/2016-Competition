package org.usfirst.frc.team1732.subsystems;

import java.util.ArrayList;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.AxisCamera;

public class Camera {
	
	private Image frame;
	private Image binaryFrame;
	private int numberParticles;
	private int session;
	//private AxisCamera camera;

	public boolean camera_exists = true; // assume it exists
	
	private final double RATIO = 1.428571;
	
	private NIVision.Range PAR_HUE_RANGE = new NIVision.Range(120, 140); // Default hue range for goal
	private NIVision.Range PAR_SAT_RANGE = new NIVision.Range(50, 255); // Default saturation range for goal
	private NIVision.Range PAR_VAL_RANGE = new NIVision.Range(150, 255); // Default value range for goal
	private double RATIO_MIN = 1.0; // goal width = 20 in. / goal height = 12 in. = 1.428
	private double RATIO_MAX = 1.8; // Goal width = 20 in. / goal height = 12 in. = 1.428
	private double MIN_AREA = 100;
	private int PAR_LIMIT = 10;
	private double direction = 0.5;
	private double distance = 0.0;
	
	public Camera() {
		//try{camera = new AxisCamera("10.99.99.9");}
		//catch(Exception e) {System.err.println("Camera not found"); camera_exists = false;}
		try{
			session = NIVision.IMAQdxOpenCamera("cam0", NIVision.IMAQdxCameraControlMode.CameraControlModeController);
			NIVision.IMAQdxConfigureGrab(session);
			NIVision.IMAQdxStartAcquisition(session);
		}
		catch(Exception e) {System.err.println("Camera not found"); camera_exists = false;}
		frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
		binaryFrame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_U8, 0);
		CameraServer.getInstance().setQuality(25);
		CameraServer.getInstance().setSize(1);
		SmartDashboard.putBoolean("binaryFrame?", false);
		SmartDashboard.putBoolean("Camera Exists?", camera_exists);
		// Color limits
		SmartDashboard.putNumber("Particle hue min", PAR_HUE_RANGE.minValue);
		SmartDashboard.putNumber("Particle hue max", PAR_HUE_RANGE.maxValue);
		SmartDashboard.putNumber("Particle sat min", PAR_SAT_RANGE.minValue);
		SmartDashboard.putNumber("Particle sat max", PAR_SAT_RANGE.maxValue);
		SmartDashboard.putNumber("Particle val min", PAR_VAL_RANGE.minValue);
		SmartDashboard.putNumber("Particle val max", PAR_VAL_RANGE.maxValue);
		// Search limits
		SmartDashboard.putNumber("Particle aspect min", RATIO_MIN);
		SmartDashboard.putNumber("Particle aspect max", RATIO_MAX);
		SmartDashboard.putNumber("Particle area min", MIN_AREA);
		SmartDashboard.putNumber("Particle Limit", PAR_LIMIT);
		SmartDashboard.putNumber("Masked particles", 0);
		SmartDashboard.putNumber("Filtered particles", 0);
		SmartDashboard.putNumber("Area", 0);
		SmartDashboard.putNumber("Aspect", 0);
		SmartDashboard.putNumber("Distance",  0);
		SmartDashboard.putNumber("Direction", 0);
		//try{camera.getImage(frame);}
		//catch(Exception e) {System.out.println("Camera not found"); camera_exists = false;}
		try{NIVision.IMAQdxGrab(session, frame, 1);}
		catch(Exception e) {System.err.println("Camera not found"); camera_exists = false;}
	}
	
	public double getAngle() {
		if( camera_exists) {
			//try{camera.getImage(frame);}
			//catch(Exception e) {System.out.println("Camera not found"); camera_exists = false;}
			try{NIVision.IMAQdxGrab(session, frame, 1);}
			catch(Exception e) {System.err.println("Camera not found"); camera_exists = false;}
			
			PAR_HUE_RANGE.minValue = (int) SmartDashboard.getNumber("Particle hue min", PAR_HUE_RANGE.minValue);
			PAR_HUE_RANGE.maxValue = (int) SmartDashboard.getNumber("Particle hue max", PAR_HUE_RANGE.maxValue);
			PAR_SAT_RANGE.minValue = (int) SmartDashboard.getNumber("Particle sat min", PAR_SAT_RANGE.minValue);
			PAR_SAT_RANGE.maxValue = (int) SmartDashboard.getNumber("Particle sat max", PAR_SAT_RANGE.maxValue);
			PAR_VAL_RANGE.minValue = (int) SmartDashboard.getNumber("Particle val min", PAR_VAL_RANGE.minValue);
			PAR_VAL_RANGE.maxValue = (int) SmartDashboard.getNumber("Particle val max", PAR_VAL_RANGE.maxValue);

			// Threshold the image looking for green (Goal color)
			NIVision.imaqColorThreshold(binaryFrame, frame, 255, NIVision.ColorMode.HSL, PAR_HUE_RANGE, PAR_SAT_RANGE, PAR_VAL_RANGE);

			// Count and display particles
			numberParticles = NIVision.imaqCountParticles(binaryFrame, 1);
			SmartDashboard.putNumber("Masked particles", numberParticles);

			// Set search limits
			RATIO_MIN = SmartDashboard.getNumber("Particle aspect min",   RATIO_MIN);
			RATIO_MAX = SmartDashboard.getNumber("Particle aspect max",   RATIO_MAX);
			MIN_AREA  = SmartDashboard.getNumber("Particle area min", MIN_AREA);
			PAR_LIMIT = (int) SmartDashboard.getNumber("Particle Limit", PAR_LIMIT);
			
			if (numberParticles > 0) {					
				ArrayList<Particle> qualifyingParticles = new ArrayList<Particle>();
				for (int particleIndex = 0; particleIndex < numberParticles; particleIndex++) {
					Particle par = new Particle(NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_TOP),
												NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT),
												NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM),
												NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT));
					double temp = par.getAspect();
					if (par.getArea() > MIN_AREA && temp < RATIO_MAX && temp > RATIO_MIN) {
						qualifyingParticles.add(par);
						//if (qualifyingParticles.size() > particleLimit) qualifyingParticles.remove(qualifyingParticles.size() - 1);
					}
				}
				SmartDashboard.putNumber("Filtered particles", qualifyingParticles.size());
				if(qualifyingParticles.size() > 0) {
					// Finds the best particle based on aspect
					Particle bestPar = qualifyingParticles.get(0);
					for (int i = 1; i < qualifyingParticles.size(); i++)
						if (Math.abs(RATIO - qualifyingParticles.get(i).getAspect()) < Math.abs(RATIO - bestPar.getAspect()))
							bestPar = qualifyingParticles.get(i);
					// Saves the direction
					direction = bestPar.getDirection();
					distance = bestPar.getDistance();
					// Displays information
					SmartDashboard.putNumber("Area", bestPar.getArea());
					SmartDashboard.putNumber("Aspect", bestPar.getAspect());
					SmartDashboard.putNumber("Distance",  distance);
					SmartDashboard.putNumber("Direction", direction);
					drawRectangle(frame, bestPar);
					if(SmartDashboard.getBoolean("binaryFrame?", false)) CameraServer.getInstance().setImage(binaryFrame);
					else CameraServer.getInstance().setImage(frame);
					return direction;
				}
				else {
					if(SmartDashboard.getBoolean("binaryFrame?", false)) CameraServer.getInstance().setImage(binaryFrame);
					else CameraServer.getInstance().setImage(frame);
					return -1.0;
				}
			} else {
				if(SmartDashboard.getBoolean("binaryFrame?", false)) CameraServer.getInstance().setImage(binaryFrame);
				else CameraServer.getInstance().setImage(frame);
				return -1.0;
			}
		}
		return -1.0;
	}
	
	public double getDistance() {
		return distance;
	}
	
	private void drawRectangle(Image image, Particle par) {
		NIVision.Rect rect = new NIVision.Rect((int) par.getTop(),
											   (int) par.getLeft(),
											   (int) (par.getBottom() - par.getTop()),
											   (int)(par.getRight() - par.getLeft()));
		NIVision.imaqDrawShapeOnImage(image,
									  image,
									  rect,
									  NIVision.DrawMode.PAINT_VALUE,
									  NIVision.ShapeMode.SHAPE_RECT, (float) 0x00FF00);
	}
	
	public void sendImage() {
		try{
			NIVision.IMAQdxGrab(session, frame, 1);
			CameraServer.getInstance().setImage(frame);
		}
		catch(Exception e) {System.err.println("Camera not found"); camera_exists = false;}
	}
	
	public void startCamera() {
		try{
			session = NIVision.IMAQdxOpenCamera("cam0", NIVision.IMAQdxCameraControlMode.CameraControlModeController);
			NIVision.IMAQdxConfigureGrab(session);
			NIVision.IMAQdxStartAcquisition(session);
		}
		catch(Exception e) {System.err.println("Camera not found"); camera_exists = false;}
	}
	
	public void stopCamera() {
		try{
			NIVision.IMAQdxStopAcquisition(session);
		}
		catch(Exception e) {System.err.println("Camera not found"); camera_exists = false;}
	}
}