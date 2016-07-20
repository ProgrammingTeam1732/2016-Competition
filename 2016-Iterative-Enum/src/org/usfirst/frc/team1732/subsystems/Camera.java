package org.usfirst.frc.team1732.subsystems;

import java.util.ArrayList;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;

import edu.wpi.first.wpilibj.CameraServer;
import org.usfirst.frc.team1732.smart_dashboard.SmartDashboardBoolean;
import org.usfirst.frc.team1732.smart_dashboard.SmartDashboardInteger;
import org.usfirst.frc.team1732.smart_dashboard.SmartDashboardDouble;

public class Camera {

	// https://wpilib.screenstepslive.com/s/4485/m/24194/l/288984-camera-settings

	public static final boolean useSmartDashboard = true;
	private SmartDashboardBoolean doesCameraExistSD = new SmartDashboardBoolean("Camera Exists?", false);

	private MyUSBCamera camera;
	String name = "cam0";
	public boolean cameraExists = false; // assume it is disconnected
	private int brightness = 0;
	private int exposure = 0;
	private int FPS = 10;
	private int whiteBalance;
	private static final int width = 320;
	private static final int height = 240;
	/*
	 * kFixedIndoor = 3000;
	 * kFixedOutdoor1 = 4000;
	 * kFixedOutdoor2 = 5000;
	 * kFixedFluorescent1 = 5100;
	 * kFixedFlourescent2 = 5200;
	 */

	private Image frame;
	private Image binaryFrame;
	private int imageCompression = 50;
	// Should experiment to see if 0 or 100 is lowest quality and how it affects speed of program
	// Note: this is only quality for sending the image to the smart dashboard

	private final double RATIO = 1.428571; // width/height, 20/12

	private NIVision.Range PAR_HUE_RANGE = new NIVision.Range(120, 140);
	private NIVision.Range PAR_SAT_RANGE = new NIVision.Range(50, 255);
	private NIVision.Range PAR_VAL_RANGE = new NIVision.Range(150, 255);
	private double RATIO_MIN = 1.1;
	private double RATIO_MAX = 1.7;
	private double MIN_AREA = 4000;
	private double distance = 0.0;
	// private int PAR_LIMIT = 10;

	public Camera() {
		try {
			camera = new MyUSBCamera(name);
		} catch (Exception e) {
			System.out.println(e.getMessage());
			System.err.println(e);
			e.printStackTrace();
			return;
		}
		frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_HSL, 0);
		binaryFrame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_U8, 0);
		doesCameraExistSD.set(true);
		// Change these values and if
		// Not sure what getting the minimum white balance value and setting it would do, if nothing try the values
		whiteBalance = (int) NIVision.IMAQdxGetAttributeMinimumI64(camera.getId(), "CameraAttributes::WhiteBalance::Value");
		camera.setWhiteBalanceManual(whiteBalance);
		camera.setExposureManual(exposure);
		camera.setBrightness(brightness);
		camera.setFPS(FPS);
		camera.setSize(width, height);
		camera.startCapture();

		if (useSmartDashboard)
			setUpSmartDashboard();
	}

	public double getAngle() {
		double angle = -1.0;
		double dist = 0.0;
		double aspect = 0.0;
		double area = 0.0;
		if (useSmartDashboard) {
			readSmartDashboardCameraSettings();
			readSmartDashboardRangeSettings();
			readSmartDashboardSearchSettings();
		}
		// Prone to errors:
		camera.getImage(frame);
		// Threshold the image looking for green (Goal color as reflected by light strip)
		NIVision.imaqColorThreshold(binaryFrame, frame, 255, NIVision.ColorMode.HSL, PAR_HUE_RANGE, PAR_SAT_RANGE, PAR_VAL_RANGE);
		// Count and display particles
		int numberParticles = NIVision.imaqCountParticles(binaryFrame, 1);
		if (useSmartDashboard)
			maskedParticles.set(numberParticles);

		if (numberParticles > 0) {
			ArrayList<Particle> qualifyingParticles = new ArrayList<Particle>();
			for (int particleIndex = 0; particleIndex < numberParticles; particleIndex++) {
				Particle par = new Particle(
						NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_TOP),
						NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT),
						NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM),
						NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT));
				double temp = par.getAspect();
				if (par.getArea() > MIN_AREA && temp < RATIO_MAX && temp > RATIO_MIN) {
					qualifyingParticles.add(par);
					// if (qualifyingParticles.size() > particleLimit)
					// qualifyingParticles.remove(qualifyingParticles.size()
					// - 1);
				}
			}
			if (useSmartDashboard)
				filteredParticles.set(qualifyingParticles.size());
			if (qualifyingParticles.size() > 0) {
				// Finds the best particle based on aspect
				Particle bestPar = qualifyingParticles.get(0);
				for (int i = 1; i < qualifyingParticles.size(); i++)
					if (Math.abs(RATIO - qualifyingParticles.get(i).getAspect()) < Math.abs(RATIO - bestPar.getAspect()))
						bestPar = qualifyingParticles.get(i);
				angle = bestPar.getDirection();
				dist = bestPar.getDistance();
				distance = dist;
				area = bestPar.getArea();
				aspect = bestPar.getAspect();
				drawRectangle(frame, bestPar);
			}
		}
		// }
		if (useSmartDashboard) {
			sendImage();
			particleArea.set(area);
			particleAspect.set(aspect);
			particleDistance.set(dist);
			particleDirection.set(angle);
		}
		return angle;
	}

	public double getDistance() {
		return distance;
	}

	private void drawRectangle(Image image, Particle par) {
		NIVision.Rect rect = new NIVision.Rect((int) par.getTop(), (int) par.getLeft(), (int) (par.getBottom() - par.getTop()),
				(int) (par.getRight() - par.getLeft()));
		NIVision.imaqDrawShapeOnImage(image, image, rect, NIVision.DrawMode.PAINT_VALUE, NIVision.ShapeMode.SHAPE_RECT, (float) 0x00FF00);
	}

	public void sendImage() {
		CameraServer.getInstance().setQuality(smartDashboardCompression.get());
		if (displayBinaryFrame.get()) {
			CameraServer.getInstance().setImage(binaryFrame);
		} else {
			CameraServer.getInstance().setImage(frame);
		}
	}

	private SmartDashboardBoolean displayBinaryFrame;
	private SmartDashboardInteger smartDashboardCompression;

	private SmartDashboardInteger particleHueMinimum;
	private SmartDashboardInteger particleHueMaximum;
	private SmartDashboardInteger particleSatMinimum;
	private SmartDashboardInteger particleSatMaximum;
	private SmartDashboardInteger particleValMinimum;
	private SmartDashboardInteger particleValMaximum;

	private SmartDashboardDouble particleAspectMinimum;
	private SmartDashboardDouble particleAspectMaximum;
	private SmartDashboardDouble particleAreaMinimum;
	// private SmartDashboardInteger qualifyingParticleLimit;
	private SmartDashboardInteger maskedParticles;
	private SmartDashboardInteger filteredParticles;
	private SmartDashboardDouble particleArea;
	private SmartDashboardDouble particleAspect;
	private SmartDashboardDouble particleDistance;
	private SmartDashboardDouble particleDirection;

	private SmartDashboardInteger cameraBrightness;
	private SmartDashboardInteger cameraExposure;
	private SmartDashboardInteger cameraFPS;
	private SmartDashboardInteger cameraWhiteBalance;

	private void setUpSmartDashboard() {
		displayBinaryFrame = new SmartDashboardBoolean("Binary frame?", false);
		smartDashboardCompression = new SmartDashboardInteger("Compression", imageCompression);

		// Color limits
		particleHueMinimum = new SmartDashboardInteger("Par hue min", PAR_HUE_RANGE.minValue);
		particleHueMaximum = new SmartDashboardInteger("Par hue max", PAR_HUE_RANGE.maxValue);
		particleSatMinimum = new SmartDashboardInteger("Par sat min", PAR_SAT_RANGE.minValue);
		particleSatMaximum = new SmartDashboardInteger("Par sat max", PAR_SAT_RANGE.maxValue);
		particleValMinimum = new SmartDashboardInteger("Par val min", PAR_VAL_RANGE.minValue);
		particleValMaximum = new SmartDashboardInteger("Par val max", PAR_VAL_RANGE.maxValue);

		// Search limits
		particleAspectMinimum = new SmartDashboardDouble("Par aspect min", RATIO_MIN);
		particleAspectMaximum = new SmartDashboardDouble("Par aspect max", RATIO_MAX);
		particleAreaMinimum = new SmartDashboardDouble("Par area min", MIN_AREA);
		// qualifyingParticleLimit = new SmartDashboardInteger("Qual Par Limit", PAR_LIMIT);
		maskedParticles = new SmartDashboardInteger("Masked particles", 0);
		filteredParticles = new SmartDashboardInteger("Filtered particles", 0);
		particleArea = new SmartDashboardDouble("Goal Area", 0);
		particleAspect = new SmartDashboardDouble("Goal Aspect", 0);
		particleDistance = new SmartDashboardDouble("Goal Distance", 0);
		particleDirection = new SmartDashboardDouble("Goal Direction", 0);

		// Camera (need variables up top)
		cameraBrightness = new SmartDashboardInteger("Camera Brightness", brightness);
		cameraExposure = new SmartDashboardInteger("Camera Exposure", exposure);
		cameraFPS = new SmartDashboardInteger("Camera FPS", FPS);
		cameraWhiteBalance = new SmartDashboardInteger("Camera White Balance", whiteBalance);
	}

	private void readSmartDashboardCameraSettings() {
		camera.setBrightness(cameraBrightness.get());
		camera.setExposureManual(cameraExposure.get());
		camera.setFPS(cameraFPS.get());
		camera.setWhiteBalanceManual(cameraWhiteBalance.get());
	}

	private void readSmartDashboardRangeSettings() {
		PAR_HUE_RANGE.minValue = particleHueMinimum.get();
		PAR_HUE_RANGE.maxValue = particleHueMaximum.get();
		PAR_SAT_RANGE.minValue = particleSatMinimum.get();
		PAR_SAT_RANGE.maxValue = particleSatMaximum.get();
		PAR_VAL_RANGE.minValue = particleValMinimum.get();
		PAR_VAL_RANGE.maxValue = particleValMaximum.get();
	}

	private void readSmartDashboardSearchSettings() {
		RATIO_MIN = particleAspectMinimum.get();
		RATIO_MAX = particleAspectMaximum.get();
		MIN_AREA = particleAreaMinimum.get();
		// PAR_LIMIT = qualifyingParticleLimit.get();
	}
}