package navigation;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class ObstacleDetector {

	// Class attributes.
	private final int bandCenter = 30;
	private final int rotateSpeed = 100;
	private final int forwardSpeed = 200;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private int distance;
	private int distError;
	private double correction;
	private double leftRadius = 2.1;
	private double rightRadius = 2.1;

	// ObstacleDetector constructor.
	public ObstacleDetector(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.distError = 0;
		this.correction = 1.0;
	}

	// processUSData allows the robot to avoid an obstacle given a distance
	// reading. (PController style)
	public void processUSData(int distance) {

		this.distance = distance;

		// If the distance is too little, instead of rotation, simply go
		// backwards by 25.
		if (distance < 12) {
			leftMotor.setSpeed(forwardSpeed - 100);
			rightMotor.setSpeed(forwardSpeed - 100);
			leftMotor.rotate(-convertDistance(leftRadius, 25), true);
			rightMotor.rotate(-convertDistance(rightRadius, 25), false);
		}
		// Else the distance is problematic since the processUSData method is
		// only called for emergencies.
		else {
			// Calculate a correction parameter depending on the actual distance
			// from the obstacle
			// The closer the obstacle, the higher the value. However, the value
			// is capped to 1.5.
			this.distError = bandCenter - distance;
			correction = 1.0 + distError / 30.0;
			if (correction > 2)
				correction = 1.5;
			// Rotate proportionally for 2 seconds.
			rightMotor.setSpeed((int) (correction * rotateSpeed));
			leftMotor.setSpeed((int) (correction * rotateSpeed));
			rightMotor.forward();
			leftMotor.backward();

			try {

				Thread.sleep(2000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			// After rotating, set the robot to go forward for 2s.
			rightMotor.setSpeed(forwardSpeed);
			leftMotor.setSpeed(forwardSpeed);
			leftMotor.forward();
			rightMotor.forward();

			try {

				Thread.sleep(2000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

		}

	}

	// Used to go backward for a specific distance (in the processUSData
	// method).
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

}
