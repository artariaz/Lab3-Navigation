package navigation;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigator extends Thread {
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor leftMotor;

	public Navigator(EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor leftMotor) {
		this.rightMotor = rightMotor;
		this.leftMotor = leftMotor;
		
	}

	public void run() {
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {
				leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}
		travelTo(60,30);
		travelTo(30,30);
		travelTo(30,60);
		travelTo(60,0);
	}

	void travelTo(double x, double y) {
		
	}

	void turnTo(double theta) {

	}

	boolean isNavigating() {

		return true;
	}
}
