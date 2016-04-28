package org.usfirst.frc.team1732.subsystems;

import java.util.ArrayList;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.AxisCamera;
import edu.wpi.first.wpilibj.vision.USBCamera;

public class Camera {

	private Image frame;
	private Image binaryFrame;
	private int numberParticles;
	//private int session;
	// private AxisCamera camera;
	private USBCamera camera;
	public boolean camera_exists = false; // assume it is disconnected

	private final double RATIO = 1.428571; // width/height, 20/12

	private NIVision.Range PAR_HUE_RANGE = new NIVision.Range(120, 140);
	private NIVision.Range PAR_SAT_RANGE = new NIVision.Range(50, 255);
	private NIVision.Range PAR_VAL_RANGE = new NIVision.Range(150, 255);
	private double RATIO_MIN = 1.1;
	private double RATIO_MAX = 1.7;
	private double MIN_AREA = 4000;
	private int PAR_LIMIT = 10;
	private double distance = 0.0;

	public Camera() {
		// try{camera = new AxisCamera("10.99.99.9");}
		// catch(Exception e) {System.err.println("Camera not found");
		// camera_exists = false;}
		try{
			frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
			binaryFrame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_U8, 0);
			camera = new USBCamera("cam1");
			camera.openCamera();
			camera.startCapture();
			camera.setWhiteBalanceManual(4000);
			SmartDashboard.putNumber("Camera Brightness: ", (int)camera.getBrightness());
			SmartDashboard.putNumber("Camera Exposure: ", 50);
			SmartDashboard.putNumber("Camera FPS: ", 10);
			//SmartDashboard.putNumber("Camera White Balance:", 0);		
		}
		catch(Exception e) {
			System.out.println(e.getMessage());
			System.err.println(e);
			e.printStackTrace();
		}
		//camera.openCamera();
		CameraServer.getInstance().setQuality(25);
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
		SmartDashboard.putNumber("Distance", 0);
		SmartDashboard.putNumber("Direction", 0);
		// try{camera.getImage(frame);}
		// catch(Exception e) {System.out.println("Camera not found");
		// camera_exists = false;}
	}

	public double getAngle() {
		double angle = -1.0;
		double dist = 0.0;
		double aspect = 0.0;
		double area = 0.0;
		try {
			//NIVision.IMAQdxGrab(session, frame, 1);
			camera.setBrightness((int) SmartDashboard.getNumber("Camera Brightness: "));
			camera.setExposureManual((int) SmartDashboard.getNumber("Camera Exposure: "));
			camera.setFPS((int) SmartDashboard.getNumber("Camera FPS: "));
			//camera.setWhiteBalanceManual((int) SmartDashboard.getNumber("Camera White Balance:"));
			camera.updateSettings();
			camera.getImage(frame);
		} catch (Exception e) {
			System.out.println(e.getMessage());
			System.err.println(e);
			e.printStackTrace();
			//camera_exists = false;
			//startCamera();
		}
		//if (camera_exists) {
			// try{camera.getImage(frame);}
			// catch(Exception e) {System.out.println("Camera not found");
			// camera_exists = false;}
			PAR_HUE_RANGE.minValue = (int) SmartDashboard.getNumber("Particle hue min", PAR_HUE_RANGE.minValue);
			PAR_HUE_RANGE.maxValue = (int) SmartDashboard.getNumber("Particle hue max", PAR_HUE_RANGE.maxValue);
			PAR_SAT_RANGE.minValue = (int) SmartDashboard.getNumber("Particle sat min", PAR_SAT_RANGE.minValue);
			PAR_SAT_RANGE.maxValue = (int) SmartDashboard.getNumber("Particle sat max", PAR_SAT_RANGE.maxValue);
			PAR_VAL_RANGE.minValue = (int) SmartDashboard.getNumber("Particle val min", PAR_VAL_RANGE.minValue);
			PAR_VAL_RANGE.maxValue = (int) SmartDashboard.getNumber("Particle val max", PAR_VAL_RANGE.maxValue);

			// Threshold the image looking for green (Goal color)
			try{
			NIVision.imaqColorThreshold(binaryFrame, frame, 255, NIVision.ColorMode.HSL, PAR_HUE_RANGE, PAR_SAT_RANGE,
					PAR_VAL_RANGE);
			}catch(Exception e){
				System.out.println(e.getMessage());
				System.err.println(e);
				e.printStackTrace();
			}

			// Count and display particles
			numberParticles = NIVision.imaqCountParticles(binaryFrame, 1);
			SmartDashboard.putNumber("Masked particles", numberParticles);

			// Set search limits
			RATIO_MIN = SmartDashboard.getNumber("Particle aspect min", RATIO_MIN);
			RATIO_MAX = SmartDashboard.getNumber("Particle aspect max", RATIO_MAX);
			MIN_AREA = SmartDashboard.getNumber("Particle area min", MIN_AREA);
			PAR_LIMIT = (int) SmartDashboard.getNumber("Particle Limit", PAR_LIMIT);

			if (numberParticles > 0) {
				ArrayList<Particle> qualifyingParticles = new ArrayList<Particle>();
				for (int particleIndex = 0; particleIndex < numberParticles; particleIndex++) {
					Particle par = new Particle(
							NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0,
									NIVision.MeasurementType.MT_BOUNDING_RECT_TOP),
							NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0,
									NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT),
							NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0,
									NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM),
							NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0,
									NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT));
					double temp = par.getAspect();
					if (par.getArea() > MIN_AREA && temp < RATIO_MAX && temp > RATIO_MIN) {
						qualifyingParticles.add(par);
						// if (qualifyingParticles.size() > particleLimit)
						// qualifyingParticles.remove(qualifyingParticles.size()
						// - 1);
					}
				}
				SmartDashboard.putNumber("Filtered particles", qualifyingParticles.size());
				if (qualifyingParticles.size() > 0) {
					// Finds the best particle based on aspect
					Particle bestPar = qualifyingParticles.get(0);
					for (int i = 1; i < qualifyingParticles.size(); i++)
						if (Math.abs(RATIO - qualifyingParticles.get(i).getAspect()) < Math
								.abs(RATIO - bestPar.getAspect()))
							bestPar = qualifyingParticles.get(i);
					angle = bestPar.getDirection();
					dist = bestPar.getDistance();
					distance = dist;
					area = bestPar.getArea();
					aspect = bestPar.getAspect();
					drawRectangle(frame, bestPar);
				}
			}
		//}
		sendImage();
		SmartDashboard.putNumber("Area", area);
		SmartDashboard.putNumber("Aspect", aspect);
		SmartDashboard.putNumber("Distance", dist);
		SmartDashboard.putNumber("Direction", angle);
		return angle;
	}
	

	public double getDistance() {
		return distance;
	}

	private void drawRectangle(Image image, Particle par) {
		NIVision.Rect rect = new NIVision.Rect((int) par.getTop(), (int) par.getLeft(),
				(int) (par.getBottom() - par.getTop()), (int) (par.getRight() - par.getLeft()));
		NIVision.imaqDrawShapeOnImage(image, image, rect, NIVision.DrawMode.PAINT_VALUE, NIVision.ShapeMode.SHAPE_RECT,
				(float) 0x00FF00);
	}

	public void sendImage() {
		try {
			if (SmartDashboard.getBoolean("binaryFrame?", false))
				CameraServer.getInstance().setImage(binaryFrame);
			else
				CameraServer.getInstance().setImage(frame);
		} catch (Exception e) {
			System.out.println(e.getMessage());
			System.err.println(e);
			e.printStackTrace();
			camera_exists = false;
		}
	}

	/*public void openCamera() {
		try {
			//session = NIVision.IMAQdxOpenCamera("cam1", NIVision.IMAQdxCameraControlMode.CameraControlModeController);
			
			/*NIVision.IMAQdxConfigureGrab(session);
			NIVision.IMAQdxSetAttributeString(session, "CameraAttributes::WhiteBalance::Mode", "Manual");
			NIVision.IMAQdxSetAttributeI64(session, "CameraAttributes::WhiteBalance::Value", NIVision.IMAQdxGetAttributeMinimumI64(session, "CameraAttributes::WhiteBalance::Value"));
			
			NIVision.IMAQdxSetAttributeString(session, "CameraAttributes::Exposure::Mode", "Manual");
			NIVision.IMAQdxSetAttributeI64(session, "CameraAttributes::Exposure::Value", NIVision.IMAQdxGetAttributeMinimumI64(session, "CameraAttributes::Exposure::Value"));
			
			NIVision.IMAQdxSetAttributeString(session, "CameraAttributes::Brightness::Mode", "Manual");
			NIVision.IMAQdxSetAttributeI64(session, "CameraAttributes::Brightness::Value", NIVision.IMAQdxGetAttributeMinimumI64(session, "CameraAttributes::Brightness::Value"));
			
			NIVision.IMAQdxStartAcquisition(session);
			camera_exists = true;
		} catch (Exception e) {
			System.out.println(e.getMessage());
			System.err.println(e);
			e.printStackTrace();
			camera_exists = false;
		}
	}*/

	/*public void startCamera() {
		try {
			//NIVision.IMAQdxStartAcquisition(session);
			camera_exists = true;
		} catch (Exception e) {
			System.out.println(e.getMessage());
			System.err.println(e);
			e.printStackTrace();
			camera_exists = false;
		}
	}
	

	public void stopCamera() {
		try {
			NIVision.IMAQdxStopAcquisition(session);
		} catch (Exception e) {
			System.out.println(e.getMessage());
			System.err.println(e);
			e.printStackTrace();
			camera_exists = false;
		}
	}*/
	
	public void startCapture() {
		camera.startCapture();
	}
}