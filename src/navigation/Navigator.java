package navigation;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
/*
 * Group: 41
 * Katy Dong      (260610798)
 * Arta Riazrafat (260636821)
 */
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
	private double error = 3;
	private double error2 = 3;
	private UltrasonicPoller ultrasonicPoller;
	private boolean part2 = false;
	EV3LargeRegulatedMotor sensorMotor;

	// Possible states while navigating
	/*
	 * INIT = initial state (sets waypoints and starts the navigation process)
	 * TURNING = Calculates the desired angle to rotate in order to get to the first coordinate and rotates the robot by that amount. 
	 * TRAVELLING = Travels forward until the waypoint (or a coordinate close to it) is reached 
	 * EMERGENCY = Used in part 2, upon the detection of an obstacle, it
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

		// index i used to stop the robot every couple of cycle
		int i = 0;
		State state = State.INIT;
		if (part2) {
			ultrasonicPoller.start();
		}
		while (true) {

			switch (state) {
			case INIT:
				if (isNavigating) {
					state = State.TURNING;
				}
				break;
			case TURNING:
				// Find out the desired angle to turn, and turn accordingly.
				getDestAngle();
				turnTo(destAngle);
				// If angle is reached, set state to travelling.
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
						// Process the emergency using obstacleDetector
						obstacleDetector.processUSData(ultrasonicPoller.getDistance());

						// Break and go to emergency state if an obstacle has
						// been detected.
						break;
					}
				} // If an emergency has not been detected or if it is Part 1,
					// check if the desired waypoint has been reached.
				if (checkIfDone(odometer.getX(), odometer.getY())) {
					// If waypoint has been reached, signal with a beep sound.
					Sound.beep();
					// And stop the motors in order to go back to the INIT state
					// and
					// navigate to the next waypoint.
					rightMotor.setSpeed(0);
					leftMotor.setSpeed(0);
					rightMotor.forward();
					leftMotor.forward();
					// Sleep the thread for a bit before proceeding to the next
					// waypoint.
					try {

						Thread.sleep(100);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					// Since a waypoint has been reached, we set isNavigating to
					// false to change the destination coordinates.
					isNavigating = false;
					state = State.INIT;
				} else {
					// If desired waypoint has not been reached, keep the robot
					// moving forward.

					// Periodically check if the angle is correct by going back
					// to TURNING state
					// every 30 times it goes through the while loop.
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
				// Once an emergency has been handled, go back to the TURNING
				// state.
				state = State.TURNING;

			}
			// Sleep for 200 ms at the end of each cycle in the while loop.
			try {

				Thread.sleep(200);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}

	// Read the ultrasonic's value with getDistance(), and if it is less than 30
	// cm, signal an emergency (return true), else
	// return false.
	public boolean checkEmergency() {
		int distance = ultrasonicPoller.getDistance();
		if (distance < 30) {
			return true;
		}
		return false;
	}

	// Using the odometer's current readings, and the destination's coordinates,
	// calculate the desired angle.
	public void getDestAngle() {
		double currentX = this.odometer.getX();
		double currentY = this.odometer.getY();
		double deltaX, deltaY;
		double angle;

		deltaX = destX - currentX;
		deltaY = destY - currentY;

		// Using the difference between the desired coordinates and the current
		// coordinates, calculate
		// the angle. (Pythagorean Theorem).

		// When either deltaX or deltaY are 0, they are special and must be
		// handled differently.
		if (Math.abs(deltaX) <= error2 && deltaY > 0) {
			angle = 0;
		} else if (Math.abs(deltaX) <= error2 && deltaY < 0) {
			angle = 180;
		} else if (Math.abs(deltaY) <= error2 && deltaX > 0) {
			angle = 90;
		} else if (Math.abs(deltaY) <= error2 && deltaX < 0) {
			angle = 270;
		}
		// If we do not have to deal with a special case, we simply calculate
		// the angle using the arctan function.
		else {
			angle = 90 - ((Math.atan(deltaY / deltaX)) * (180 / Math.PI));

		}

		// Once the angle has been calculated, we set destAngle to it.
		this.destAngle = angle;
	}

	// travelTo sets the desired position and starts the navigation process by
	// setting isNavigating to true.
	void travelTo(double x, double y) {
		this.destX = x;
		this.destY = y;
		getDestAngle();
		this.isNavigating = true;

	}

	// Looks at current odometer readings and the desired values, and returns
	// true if they are within the error range.
	public boolean checkIfDone(double x, double y) {
		// x and y are the odometer's readings
		// Compare with destX and destY with a degree of tolerance
		// And return true if they are close to the desired values
		if ((destX + error >= x && destX - error <= x) && (destY + error >= y && destY - error <= y)) {

			return true;
		} else
			return false;
	}

	// Looks at the current odometer angle reading and compares it with the
	// desired angle reading.
	public boolean facingDest() {
		double currentAngle = this.odometer.getTheta();

		// If angle is near the desired angle given a tolerance error, return
		// true.
		if (destAngle + error >= currentAngle || destAngle - error <= currentAngle) {
			return true;
		} else
			return false;
	}

	// updateTravel() sets the robot to go forward (movement duration is set by
	// the thread.sleep in the while loop).
	void updateTravel() {
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.setSpeed(FORWARD_SPEED);

		rightMotor.forward();
		leftMotor.forward();
	}

	// turnTo rotates the robot to a desired angle (with respect to its current
	// angle) and tries to minimize the rotational angle.
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

		// Sets the thread to sleep for 200 ms once the robot finishes rotating.
		try {

			Thread.sleep(200);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

	// Simply returns the class's isNavigating value.
	boolean isNavigating() {

		return this.isNavigating;
	}

	// convertDistance and convertAngle are used to rotate the motors in the
	// turnTo method.
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	// Setter method for the part2 boolean value.
	public void setPart2(boolean val) {
		this.part2 = val;
	}

}
