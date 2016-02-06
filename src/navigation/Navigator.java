package navigation;

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

	enum State {INIT, TURNING, TRAVELLING};
	
	public Navigator(EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor leftMotor, Odometer odometer, double rightRadius, double leftRadius, double width) {
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
					state = State.TRAVELLING;
				}
				break;
			case TRAVELLING:
				if (!checkIfDone(odometer.getX(), odometer.getY()){
					updateTravel();
				}
				else {
					rightMotor.stop();
					leftMotor.stop();
					isNavigating = false;
					state = State.INIT;
				}
				break;
			}
			try {
				Thread.sleep(30);
			} catch (InterrupedException e) {
				e.printStackTrace();
			}
		}
	}
	
	public double getDestAngle() {
		double currentX = this.odometer.getX();
		double currentY = this.odometer.getY();
		double deltaX, deltaY;

		deltaX = Math.abs(currentX - destX);
		deltaY = Math.abs(currentY - destY);

		destAngle = 90 - ((Math.atan(deltaY / deltaX)) * (180 / Math.PI));
		return destAngle;
	}

	

	void travelTo(double x, double y) {
		this.destX = x;
		this.destY = y;
		this.destAngle = getDestAngle();
		this.isNavigating = true;

	}
	
	public boolean checkIfDone(double [] position) {
		return true;
	}
	
	public boolean facingDest() {
		return true;
	}
	
	
	void updateTravel() {
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.setSpeed(FORWARD_SPEED);
		
		rightMotor.forward();
		leftMotor.forward();
	}

	void turnTo(double desiredTheta) {
		double theta = this.odometer.getTheta();
		double rotationTheta;
		double smallestTheta;
		if (theta > desiredTheta) {
			rotationTheta = theta - desiredTheta;
			if (rotationTheta > 180) {
				// Turn left by 360 - rotationTheta
				smallestTheta = 360 - rotationTheta;
			} else {
				// Turn right by rotationTheta
				smallestTheta = rotationTheta;
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);

		//		leftMotor.rotate(convertAngle(leftRadius, width, 90.0), true);
		//		rightMotor.rotate(-convertAngle(rightRadius, width, 90.0), false);
			}

		} else if (theta < desiredTheta) {
			rotationTheta = desiredTheta - theta;
			if (rotationTheta > 180) {
				// Turn right by 360 - rotationTheta
				smallestTheta = 360 - rotationTheta;
			} else {
				// Turn left by rotationTheta
				smallestTheta = rotationTheta;
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
