package navigation;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
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
		Navigator navigator = new Navigator(rightMotor, leftMotor, odometer, WHEEL_RADIUS, WHEEL_RADIUS, TRACK);

		do {
			// clear the display
			t.clear();

			// ask the user whether the motors should drive in a square or float
			t.drawString("< Left | Right >", 0, 0);
			t.drawString("       |        ", 0, 1);
			t.drawString(" Float | Drive  ", 0, 2);
			t.drawString("motors | in a   ", 0, 3);
			t.drawString("       | square ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {

			leftMotor.forward();
			leftMotor.flt();
			rightMotor.forward();
			rightMotor.flt();

			odometer.start();
			odometryDisplay.start();

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
		int[][] waypoints = { { 60, 30 }, { 30, 30 }, { 30, 60 }, { 60, 0 } };
		for (int[] point : waypoints) {
			nav.travelTo(point[0], point[1]);
			while (nav.isNavigating()) {
				Thread.sleep(500);
			}
		}

	}

}
