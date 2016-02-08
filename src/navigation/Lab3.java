package navigation;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import navigation.Odometer;
import navigation.OdometerDisplay;

/*
 * Group: 41
 * Katy Dong      (260610798)
 * Arta Riazrafat (260636821)
 */

public class Lab3 {

	// Static Resources:
	// Left motor connected to output A
	// Right motor connected to output C
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	
	private static final Port usPort = LocalEV3.get().getPort("S1");

	// Constants
	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 15.6;

	public static void main(String[] args) {
		int buttonChoice;

		// some objects that need to be instantiated

		final TextLCD t = LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(leftMotor, rightMotor, WHEEL_RADIUS, TRACK);
		OdometerDisplay odometryDisplay = new OdometerDisplay(odometer, t);
		// OdometryCorrection odometryCorrection = new
		// OdometryCorrection(odometer);
		ObstacleDetector obstacleDetector = new ObstacleDetector(leftMotor, rightMotor);
		SensorModes usSensor = new EV3UltrasonicSensor(usPort);		// usSensor is the instance
		SampleProvider us = usSensor.getMode("Distance");	// usDistance provides samples from this instance
		float[] usData = new float[us.sampleSize()];
		Navigator navigator = new Navigator(rightMotor, leftMotor, odometer, WHEEL_RADIUS, WHEEL_RADIUS, TRACK, us, usData, obstacleDetector);

		do {
			// clear the display
			t.clear();

			// ask the user whether the motors should drive in a square or float
			t.drawString("< Left | Right >", 0, 0);
			t.drawString("       |        ", 0, 1);
			t.drawString(" Part2 |  Part1 ", 0, 2);
			
			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {

			//change this when finish obstacle detector
			leftMotor.forward();
			leftMotor.flt();
			rightMotor.forward();
			rightMotor.flt();

			odometer.start();
			odometryDisplay.start();
			//obstacleDetector.start();

		} else {
			// start the odometer, the odometry display and (possibly) the
			// odometry correction

			odometer.start();
			odometryDisplay.start();
			navigator.start();
			try {
				completeCourse(navigator);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}

	public static void completeCourse(Navigator nav) throws InterruptedException {
		//int[][] waypoints = { { 60, 30 }, { 30, 30 }, { 30, 60 }, { 60, 0 } };	//use for navigation
		int [][] waypoints = {{0,60}, {60,0}};	//use for Obstacle Detection
		for (int[] point : waypoints) {
			nav.travelTo(point[0], point[1]);
			while (nav.isNavigating()) {
				Thread.sleep(500);
			}
		}

	}

}
