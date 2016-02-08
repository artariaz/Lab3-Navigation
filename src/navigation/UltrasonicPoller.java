package navigation;
import lejos.robotics.SampleProvider;

//
//  Control of the wall follower is applied periodically by the 
//  UltrasonicPoller thread.  The while loop at the bottom executes
//  in a loop.  Assuming that the us.fetchSample, and cont.processUSData
//  methods operate in about 20mS, and that the thread sleeps for
//  50 mS at the end of each loop, then one cycle through the loop
//  is approximately 70 mS.  This corresponds to a sampling rate
//  of 1/70mS or about 14 Hz.
//


public class UltrasonicPoller extends Thread{
	private SampleProvider us;
	private ObstacleDetector obstacleDetector;
	private float[] usData;
	private int distance;

	
	public UltrasonicPoller(SampleProvider us, float[] usData, ObstacleDetector obstacleDetector) {
		this.us = us;
		this.obstacleDetector = obstacleDetector;
		this.usData = usData;
	}

//  Sensors now return floats using a uniform protocol.
//  Need to convert US result to an integer [0,255]
	
	public void run() {
		while (true) {
			us.fetchSample(usData,0);							// acquire data
			distance=(int)(usData[0]*100.0);					// extract from buffer, cast to int
			obstacleDetector.processUSData(distance);						// now take action depending on value
			try { Thread.sleep(25); } catch(Exception e){}		// Poor man's timed sampling
		}
	}
	
}