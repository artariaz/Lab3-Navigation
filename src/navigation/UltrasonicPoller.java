package navigation;

import lejos.robotics.SampleProvider;

public class UltrasonicPoller extends Thread {
	private SampleProvider us;
	private float[] usData;
	private int distance = 0;

	public UltrasonicPoller(SampleProvider us, float[] usData) {
		this.us = us;
		this.usData = usData;
	}

	// Sensors now return floats using a uniform protocol.
	// Need to convert US result to an integer [0,255]

	public void run() {
		while (true) {
			us.fetchSample(usData, 0); // acquire data
			distance = (int) (usData[0] * 100.0); // extract from buffer, cast
													// to int
			try {
				Thread.sleep(100);
			} catch (Exception e) {
			} // Poor man's timed sampling
		}
	}

	// getDistance method used to the distance reading of the US sensor.
	public int getDistance() {
		return this.distance;
	}
}