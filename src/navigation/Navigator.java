package navigation;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class Navigator extends Thread {

	// Resources required to navigate robot
	private static final int FORWARD_SPEED = 100;
	private static final int ROTATE_SPEED = 125;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor leftMotor;
	private ObstacleDetector obstacleDetector;
	private boolean isNavigating = false;
	private Odometer odometer;
	private double rightRadius, leftRadius, width;
	private double destX, destY, destAngle;
	// Error values used to tolerate ranges of values instead of single values
	private double error = 2;
	private double error2 = 5;
	private UltrasonicPoller ultrasonicPoller;
	private boolean part2 = false;
	EV3LargeRegulatedMotor sensorMotor;

	// Possible states while navigating
	/*
	 * INIT = initial state (sets waypoints and starts the navigation process)
	 * TURNING = Calculates the desired angle to rotate in order to get to the
	 * first coordinate and rotates the robot by that amount. TRAVELLING =
	 * Travels forward until the waypoint (or a coordinate close to it) is
	 * reached EMERGENCY = Used in part 2, upon the detection of an obstacle, it
	 * changes its path to avoid it.
	 */
	enum State {
		INIT, TURNING, TRAVELLING, EMERGENCY
	};

	// Navigator constructor
	public Navigator(EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor leftMotor, Odometer odometer,
			double rightRadius, double leftRadius, double width, ObstacleDetector obstacleDetector,
			UltrasonicPoller ultrasonicPoller, EV3LargeRegulatedMotor sensorMotor) {

		this.rightMotor = rightMotor;
		this.leftMotor = leftMotor;
		this.odometer = odometer;
		this.leftRadius = leftRadius;
		this.rightRadius = rightRadius;
		this.width = width;
		this.obstacleDetector = obstacleDetector;
		this.ultrasonicPoller = ultrasonicPoller;
		this.sensorMotor = sensorMotor;

	}

	// Starts the navigator thread
	public void run() {
		// Goes from state to state and calls the necessary methods in each
		// state.
		int i = 0;
		State state = State.INIT;
		if (part2) {
			ultrasonicPoller.start();
			sensorTurn();
		}
		while (true) {

			switch (state) {
			case INIT:
				if (isNavigating) {
					state = State.TURNING;
				}
				break;
			case TURNING:
				this.destAngle = getDestAngle();
				turnTo(destAngle);
				if (facingDest()) {
					state = State.TRAVELLING;
					Sound.beep();
				}
				break;
			case TRAVELLING:
				// Look for emergency if it's part 2
				if (part2) {
					if (checkEmergency()) {
						// Beeps used to signal emergency state
						Sound.twoBeeps();
						Sound.twoBeeps();
						state = State.EMERGENCY;
						obstacleDetector.processUSData(ultrasonicPoller.getDistance());
						
						// Break and go to emergency state if an obstacle has
						// been detected
						break;
					}
				} // If it an emergency has not been detected or if it's part 1,
					// check if the desired waypoint has been reached
				if (checkIfDone(odometer.getX(), odometer.getY())) {
					// If waypoint has been reached, signal with a beep sound.
					Sound.beep();
					// And stop the motors to go back to the INIT state and
					// navigate to the next waypoint
					rightMotor.stop();
					leftMotor.stop();
					// Sleep the thread for a bit before proceding to the next
					// waypoint
					try {

						Thread.sleep(100);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					isNavigating = false;
					state = State.INIT;
				} else {
					// If desired waypoint has not been reached, keep the robot
					// moving forward.
					if (i < 30) {
						i++;
						updateTravel();
					} else {
						state = State.TURNING;
						i = 0;
					}
				}
				break;
			case EMERGENCY:
				state = State.TURNING;

			}
			try {

				Thread.sleep(100);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}

	public boolean checkEmergency() {
		int distance = ultrasonicPoller.getDistance();
		if (distance < 20) {
			return true;
		}
		return false;
	}

	public double getDestAngle() {
		double currentX = this.odometer.getX();
		double currentY = this.odometer.getY();
		double deltaX, deltaY;

		deltaX = destX - currentX;
		deltaY = destY - currentY;

		if (Math.abs(deltaX) <= error2 && deltaY > 0) {
			destAngle = 0;
		} else if (Math.abs(deltaX) <= error2 && deltaY < 0) {
			destAngle = 180;
		} else if (Math.abs(deltaY) <= error2 && deltaX > 0) {
			destAngle = 90;
		} else if (Math.abs(deltaY) <= error2 && deltaX < 0) {
			destAngle = 270;
		} else {
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
		if ((destX + error >= x && destX - error <= x) && (destY + error >= y && destY - error <= y)) {

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
		/*
		 * try {
		 * 
		 * Thread.sleep(5); } catch (InterruptedException e) { // TODO
		 * Auto-generated catch block e.printStackTrace(); }
		 */
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
				leftMotor.rotate(convertAngle(leftRadius, width, smallestAngle), true);
				rightMotor.rotate(-convertAngle(rightRadius, width, smallestAngle), false);
			} else {
				// Turn left by rotationAngle
				smallestAngle = rotationAngle;
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(-convertAngle(leftRadius, width, smallestAngle), true);
				rightMotor.rotate(convertAngle(rightRadius, width, smallestAngle), false);
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
		try {

			Thread.sleep(200);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
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

	public void setPart2(boolean val) {
		this.part2 = val;
	}

	public void sensorTurn() {
		(new Thread() {
			public void run() {
				while (true) {
					sensorMotor.setSpeed(50);
					sensorMotor.rotate(40);
					try {

						Thread.sleep(50);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					sensorMotor.rotate(-40);
					try {

						Thread.sleep(50);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
			}
		}).start();

	}
}
