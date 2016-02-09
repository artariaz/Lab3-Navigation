package navigation;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class ObstacleDetector {

	private final int bandCenter = 10, bandwidth = 2;
	private final int motorStraight = 150, FILTER_OUT = 20;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private int distance;
	private int filterControl;
	private int distError;
	private double correction;
	private double leftRadius = 2.1;
	private double rightRadius = 2.1;

	public ObstacleDetector(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		// Default Constructor
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		filterControl = 0;
		this.distError = 0;
		this.correction = 1.0;
	}

	public void processUSData(int distance) {

		this.distance = distance;
		// TODO: process a movement based on the us distance passed in
		// (BANG-BANG style)
		// Sensor on left, wall on left
		this.distError = bandCenter - distance;
		correction = 1.0 + distError / 20.0;
		if (correction > 2)
			correction = 2;
		rightMotor.setSpeed((int) (correction * motorStraight));
		leftMotor.setSpeed((int) (correction * 30));
		rightMotor.forward();
		leftMotor.backward();

		try {

			Thread.sleep(6000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		leftMotor.setSpeed(motorStraight);
		rightMotor.setSpeed(motorStraight);

		leftMotor.rotate(convertDistance(leftRadius, 30), true);
		rightMotor.rotate(convertDistance(rightRadius, 30), false);

	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

}
