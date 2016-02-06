package navigation;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigator extends Thread {
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor leftMotor;
	private boolean state = false;
	private Odometer odometer;

	public Navigator(EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor leftMotor, Odometer odometer) {
		this.rightMotor = rightMotor;
		this.leftMotor = leftMotor;
		this.odometer = odometer;

	}

	public void run() {
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}

		
		travelTo(60,30);
		travelTo(30,30);
		travelTo(30,60);
		travelTo(60,0);
	}

	void travelTo(double x, double y) {
		double currentX = this.odometer.getX();
		double currentY = this.odometer.getY();
		double deltaX, deltaY;
		double desiredTheta;
		
		deltaX = Math.abs(currentX - x);
		deltaY = Math.abs(currentY - y);
		
		desiredTheta = (Math.atan(deltaX/deltaY)) * (180/Math.PI);
		turnTo(desiredTheta);
		
	}

	void turnTo(double desiredTheta) {
		double theta = this.odometer.getTheta();
		double rotationTheta;

		if (theta > desiredTheta) {
			rotationTheta = theta - desiredTheta;
			if (rotationTheta > 180) {
				// Turn left by 360 - rotationTheta
			} else {
				// Turn right by rotationTheta
			}

		} else if (theta < desiredTheta) {
			rotationTheta = desiredTheta - theta;
			if (rotationTheta > 180) {
				// Turn right by 360 - rotationTheta
			} else {
				// Turn left by rotationTheta
			}
		}

	}

	boolean isNavigating() {

		return this.state;
	}
}
