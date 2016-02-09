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
	// Ultrasonic sensor connected to port 1 (S1)
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	private static final EV3LargeRegulatedMotor sensorMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final Port usPort = LocalEV3.get().getPort("S1");

	// Constants used for odometry and navigating
	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 15.6;

	public static void main(String[] args) {
		int buttonChoice;

		// some objects that need to be instantiated

		final TextLCD t = LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(leftMotor, rightMotor, WHEEL_RADIUS, TRACK);
		ObstacleDetector obstacleDetector = new ObstacleDetector(leftMotor, rightMotor);
		@SuppressWarnings("resource")
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is
																// the instance
		SampleProvider us = usSensor.getMode("Distance"); // usDistance provides
															// samples from this
															// instance
		float[] usData = new float[us.sampleSize()];
		UltrasonicPoller ultrasonicPoller = new UltrasonicPoller(us, usData);
		OdometerDisplay odometryDisplay = new OdometerDisplay(odometer, ultrasonicPoller, t);
		Navigator navigator = new Navigator(rightMotor, leftMotor, odometer, WHEEL_RADIUS, WHEEL_RADIUS, TRACK,
				obstacleDetector, ultrasonicPoller, sensorMotor);

		do {
			// clear the display
			t.clear();

			// ask the user whether it is for part 1 or part 2 of the demo
			t.drawString("< Left | Right >", 0, 0);
			t.drawString("       |        ", 0, 1);
			t.drawString(" Part2 |  Part1 ", 0, 2);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		// Left button selects part 2 settings (Obstacle Avoiding)
		if (buttonChoice == Button.ID_LEFT) {

			// Start threads required for part 2
			odometer.start();
			odometryDisplay.start();
			// setPart2 boolean allows the use ultrasonicpoller in navigation
			// (which is not required for part 1)
			navigator.setPart2(true);
			navigator.start();

			// Complete part 2, if there is an error, print it on the screen.
			try {
				completeCourseObstacle(navigator);
			} catch (InterruptedException e) {

				e.printStackTrace();
			}

		} else {
			// start threads required for part 1
			odometer.start();
			odometryDisplay.start();
			navigator.setPart2(false);
			navigator.start();
			// Complete part 1, if there is an error, print it on the screen.
			try {
				completeCourse(navigator);
			} catch (InterruptedException e) {

				e.printStackTrace();
			}

		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}

	// This method gives waypoints coordinates for navigating in part 1
	public static void completeCourse(Navigator nav) throws InterruptedException {
		int[][] waypoints = { { 60, 30 }, { 30, 30 }, { 30, 60 }, { 60, 0 } }; // use
																				// for
																				// navigation
		for (int[] point : waypoints) {
			nav.travelTo(point[0], point[1]);
			while (nav.isNavigating()) {
				Thread.sleep(500);
			}
		}

	}

	// This method gives waypoints coordinates for navigating in part 2
	public static void completeCourseObstacle(Navigator nav) throws InterruptedException {
		int[][] waypoints = { { 0, 60 }, { 60, 0 } }; // use for Obstacle
														// Detection
		for (int[] point : waypoints) {
			nav.travelTo(point[0], point[1]);
			while (nav.isNavigating()) {
				Thread.sleep(500);
			}
		}

	}

}
