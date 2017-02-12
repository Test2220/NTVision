package org.usfirst.frc.team2220.robot;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.utils.Converters;

/**
 * This is a demo program showing the use of OpenCV to do vision processing. The
 * image is acquired from the Axis camera, then a rectangle is put on the image and
 * sent to the dashboard. OpenCV has many methods for different types of
 * processing.
 */
public class Robot extends IterativeRobot {
	Joystick stick = new Joystick(0);
	boolean isAiming = false;
	NetworkTable contourTable;
	
	@Override
	public void robotInit()
	{
		RobotMap.init();
		contourTable = NetworkTable.getTable("GRIP/contoursReport");
	}
	
	
	@Override
	public void teleopPeriodic() 
	{
		if(stick.getRawButton(1)) //button.onPress(btnA)
		{
			isAiming = true;
		}
		else
		{
			isAiming = false;
		}
		
		if(isAiming)
		{
			double[] centerX = contourTable.getNumberArray("centerX", new double[]{});
		
			//double minX = 320, maxX = 0; //lol
			System.out.println("Contour 1");
			if(centerX.length >= 1)
			{
				//List<Point> points = new ArrayList<Point>();
				//Converters.Mat_to_vector_Point(centerX[0], points);
				/*
				for(Point point: points)
				{
					if(point.x < minX)
						minX = point.x;
					if(point.x > maxX)
						maxX = point.x;
				}
				*/
			//	System.out.println("Max = " + maxX + " Min = " + minX);
				double imageMiddle = 160;
				//double size = maxX - minX;
			//	double midPoint = ((size) / 2) + minX;
				double midPoint = centerX[0];
				System.out.println("Image middle = 160");
				System.out.println("Point middle = " + midPoint);
				
				double RTLVal = imageMiddle - midPoint;
				
				double range = 20;
				double motorPower = 0.2;
				if(RTLVal > range)
				{
					RobotMap.rightMaster.set(-motorPower);
					RobotMap.leftMaster.set(-motorPower); 
				}
				else if(RTLVal < -range)
				{
					RobotMap.rightMaster.set(motorPower);
					RobotMap.leftMaster.set(motorPower); 
				}
				else
				{
					RobotMap.rightMaster.set(0);
					RobotMap.leftMaster.set(0);
				}
				
			}
			//robotUsingContourList = false;
		}
		else //standard tank
		{
			double lVal = stick.getRawAxis(1) * -1;
			double rVal = stick.getRawAxis(5);
			RobotMap.rightMaster.set(rVal);
			RobotMap.leftMaster.set(lVal);
		}
	
	}
}


//Thread visionThread;

	//public boolean cameraProcessing = false;
	
	//List<MatOfPoint> contours = new ArrayList<>();
	//boolean cameraUsingContourList = true;
	//boolean robotUsingContourList = false;
	
	/*
	//Image Processing
	UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
  CvSink cvSink = CameraServer.getInstance().getVideo();
  CvSource contourStream  = CameraServer.getInstance().putVideo("Contours", 320, 240);
  Mat processedFrame = new Mat();
  Mat contourMat = new Mat();
  
	public void processFrame()
	{
		//GRIP -> https://github.com/WPIRoboticsProjects/GRIP/releases
		//read WPI documentation: https://wpilib.screenstepslive.com/s/4485/m/24194
		//google -> "site:http://docs.opencv.org" [question e.g. morphology, filtering, HSV, BGR] 
		cvSink.grabFrame(contourMat);

      cvSink.grabFrame(processedFrame);

      // convert the frame to HSV
      Imgproc.cvtColor(processedFrame, processedFrame, Imgproc.COLOR_BGR2HSV);
      
      Scalar lowerB = new Scalar(0,150,150);
      Scalar upperB = new Scalar(255,255,255);
      Core.inRange(processedFrame, lowerB, upperB, processedFrame);
      
      // init
      contours = new ArrayList<MatOfPoint>();

      Mat hierarchy = new Mat();

      // find contours
      Imgproc.findContours(processedFrame, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

      // if any contour exist...
      if (hierarchy.size().height > 0 && hierarchy.size().width > 0)
      {
              // for each contour, display it in red
              for (int idx = 0; idx >= 0; idx = (int) hierarchy.get(0, idx)[0])
              {
                      Imgproc.drawContours(contourMat, contours, idx, new Scalar(0, 0, 250));
              }
      }
      contourStream.putFrame(contourMat);
	}
	*/
/*
camera.setResolution(320, 240);
camera.setExposureManual(15);

new Thread(() -> //continually feed normal camera to dash, if image processing not taking place
{
	while(!Thread.interrupted()) 
	{
		if(!cameraProcessing)
		{
			cvSink.grabFrame(contourMat);
			contourStream.putFrame(contourMat);
			
		}
	}
}).start();
*/

/* old methodology
@Override
public void robotInit() {
RobotMap.init();
new Thread(() -> {
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(320, 240);
    camera.setExposureManual(15);
    CvSink cvSink = CameraServer.getInstance().getVideo();
    CvSource contourStream  = CameraServer.getInstance().putVideo("Contours", 320, 240);
    
    Mat processedFrame = new Mat();
    Mat contourMat = new Mat();
    
    while(!Thread.interrupted()) {
    	cvSink.grabFrame(contourMat);
    	if(cameraProcessing && !robotUsingContourList)
    	{
            cvSink.grabFrame(processedFrame);

            // convert the frame to HSV
            Imgproc.cvtColor(processedFrame, processedFrame, Imgproc.COLOR_BGR2HSV);
            
            Scalar lowerB = new Scalar(0,150,150);
            Scalar upperB = new Scalar(255,255,255);
            Core.inRange(processedFrame, lowerB, upperB, processedFrame);
            
            // init
            cameraUsingContourList = true;
            contours = new ArrayList<MatOfPoint>();
            
            
            Mat hierarchy = new Mat();
            
            
            // find contours
            Imgproc.findContours(processedFrame, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

            // if any contour exist...
            if (hierarchy.size().height > 0 && hierarchy.size().width > 0)
            {
                    // for each contour, display it in blue
                    for (int idx = 0; idx >= 0; idx = (int) hierarchy.get(0, idx)[0])
                    {
                            Imgproc.drawContours(contourMat, contours, idx, new Scalar(0, 0, 250));
                    }
            }
            cameraUsingContourList = false;
    	}
        contourStream.putFrame(contourMat);
    }
}).start();
}
*/
