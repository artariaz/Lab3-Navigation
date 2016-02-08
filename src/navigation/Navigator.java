package navigation;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigator extends Thread {
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor leftMotor;
	private boolean isNavigating = false;
	private Odometer odometer;
	private double rightRadius, leftRadius, width;
	private double destX, destY, destAngle;
	private double error = 3;

	enum State {
		INIT, TURNING, TRAVELLING
	};

	public Navigator(EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor leftMotor, Odometer odometer,
			double rightRadius, double leftRadius, double width) {
		this.rightMotor = rightMotor;
		this.leftMotor = leftMotor;
		this.odometer = odometer;
		this.leftRadius = leftRadius;
		this.rightRadius = rightRadius;
		this.width = width;

	}

	public void run() {
		State state = State.INIT;
		while (true) {
			switch (state) {
			case INIT:
				if (isNavigating) {
					state = State.TURNING;
				}
				break;
			case TURNING:
				turnTo(destAngle);
				if (facingDest()) {
					Sound.beep();
					state = State.TRAVELLING;
				}
				break;
			case TRAVELLING:
				if (!checkIfDone(odometer.getX(), odometer.getY())) {
					updateTravel();
				} else {
					Sound.twoBeeps();
					rightMotor.stop();
					leftMotor.stop();
					isNavigating = false;
					state = State.INIT;
				}
				break;
			}
			try {
				Thread.sleep(30);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

	}

	public double getDestAngle() {
		double currentX = this.odometer.getX();
		double currentY = this.odometer.getY();
		double deltaX, deltaY;

		deltaX = destX-currentX;
		deltaY = destY-currentY;
		
		if (Math.abs(deltaX) <= error  && deltaY >0) {
			destAngle = 0;
		}
		else if (Math.abs(deltaX) <= error && deltaY<0) {
			destAngle = 180;
		}
		else if (Math.abs(deltaY) <= error && deltaX > 0) {
			destAngle = 90;
		}
		else if (Math.abs(deltaY) <= error && deltaX<0) {
			destAngle = 270;
		}
		else {
			destAngle = 90 - ((Math.atan(deltaY / deltaX)) * (180 / Math.PI));
		
		}
		return destAngle;
	}

	void travelTo(double x, double y) {
		this.destX = x;
		this.destY = y;
		this.destAngle = getDestAngle();
		this.isNavigating = true;

	}

	public boolean checkIfDone(double x, double y) {
		// x and y are the odometer's readings
		// Compare with destX and destY with a degree of tolerance
		// And return true if they are close to the desired values
		if (destX + error >= x && destX - error <= x && destY + error >= y && destY - error <= y) {
			return true;
		} else
			return false;
	}

	public boolean facingDest() {
		double currentAngle = this.odometer.getTheta();

		// If angle is near the desired angle given a tolerance error, return
		// true
		if (destAngle + error >= currentAngle || destAngle - error <= currentAngle) {
			return true;
		} else
			return false;
	}

	void updateTravel() {
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.setSpeed(FORWARD_SPEED);

		rightMotor.forward();
		leftMotor.forward();
		try {
			
			Thread.sleep(200);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	void turnTo(double desiredAngle) {
		double currentAngle = this.odometer.getTheta();
		double rotationAngle;
		double smallestAngle;
		if (currentAngle > desiredAngle) {
			rotationAngle = currentAngle - desiredAngle;
			if (rotationAngle > 180) {
				// Turn right by 360 - rotationAngle
				smallestAngle = 360 - rotationAngle;
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(-convertAngle(leftRadius, width, smallestAngle), true);
				rightMotor.rotate(convertAngle(rightRadius, width, smallestAngle), false);
			} else {
				// Turn left by rotationAngle
				smallestAngle = rotationAngle;
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(convertAngle(leftRadius, width, smallestAngle), true);
				rightMotor.rotate(-convertAngle(rightRadius, width, smallestAngle), false);
			}

		} else if (currentAngle < desiredAngle) {
			rotationAngle = desiredAngle - currentAngle;
			if (rotationAngle > 180) {
				// Turn left by 360 - rotationTheta
				smallestAngle = 360 - rotationAngle;
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(-convertAngle(leftRadius, width, smallestAngle), true);
				rightMotor.rotate(convertAngle(rightRadius, width, smallestAngle), false);
			} else {
				// Turn right by rotationTheta
				smallestAngle = rotationAngle;
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(convertAngle(leftRadius, width, smallestAngle), true);
				rightMotor.rotate(-convertAngle(rightRadius, width, smallestAngle), false);
			}
		}

	}

	boolean isNavigating() {

		return this.isNavigating;
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
