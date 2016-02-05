package navigation;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends Thread {
	// robot position
	private double x, y, theta;

	// odometer update period, in ms
	private static final long ODOMETER_PERIOD = 25;

	// lock object for mutual exclusion
	private Object lock;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private double wheelRadius, width;
	private int lastTachoL;
	private int lastTachoR;
	private int nowTachoL;
	private int nowTachoR;
	private double lDist, rDist, deltaDist, deltaTheta, dX, dY;
	private double thetaRad = 0;

	// default constructor
	// Motors, radius and width are passed in as parameters from the Lab2 class.
	public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double wheelRadius,
			double width) {
		x = 0.0;
		y = 0.0;
		theta = 0.0;
		lock = new Object();
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.wheelRadius = wheelRadius;
		this.width = width;
	}

	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		// reset the tacho count for the motors so that it starts at 0
		leftMotor.resetTachoCount();
		rightMotor.resetTachoCount();
		// set the last tacho count to 0, which is the resetted value
		lastTachoL = leftMotor.getTachoCount();
		lastTachoR = rightMotor.getTachoCount();

		while (true) {
			updateStart = System.currentTimeMillis();
			// put (some of) your odometer code here

			nowTachoL = leftMotor.getTachoCount();
			nowTachoR = rightMotor.getTachoCount();

			// get the distance travelled by the right and left wheels using
			// their tacho count
			lDist = Math.PI * wheelRadius * (nowTachoL - lastTachoL) / 180;
			rDist = Math.PI * wheelRadius * (nowTachoR - lastTachoR) / 180;
			// update the last tacho count
			lastTachoL = nowTachoL;
			lastTachoR = nowTachoR;
			// get the distance travelled by the robot and the change in the
			// angle
			deltaDist = 0.5 * (lDist + rDist);
			deltaTheta = ((lDist - rDist) / width);
			// add the change in angle to the current angle
			// thetaRad is the angle in radians (sine and cosine take in angles
			// in radians)
			thetaRad += deltaTheta;
			dX = deltaDist * Math.sin(thetaRad);
			dY = deltaDist * Math.cos(thetaRad);

			synchronized (lock) {
				// don't use the variables x, y, or theta anywhere but here!

				// calculate the current angle and convert to degrees
				theta += (deltaTheta * (180 / Math.PI));
				// the x and y values are updated using the calculations above
				x = x + dX;
				y = y + dY;
				// the angle is bounded (0, 360]
				if (theta > 360) {
					theta = theta - 360;
				}
				if (theta < 0) {
					theta = theta + 360;
				}

			}

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}

	// accessors
	public void getPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = theta;
		}
	}

	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}

	// mutators
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}

	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

	public void setTheta(double theta) {
		synchronized (lock) {
			this.theta = theta;
		}
	}

}
