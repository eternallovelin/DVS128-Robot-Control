package jaer.myjaer.neuron;

import java.awt.geom.Point2D;

public class Signal
{
	private int label;
	private int polarity;
	private int expectedArrivalTime;
	private float expectedVelocity; 
	private int priority;
	private int counter;
	private int timestamp;
	private Point2D.Float location;
	private	int tier;
	
	public Signal(int expectedArrivalTime, float expectedVelocity, int priority, int signal_counter, int counter, int timestamp, Point2D.Float location, int polarity)
	{
		this.setExpectedArrivalTime(expectedArrivalTime);
		this.setExpectedVelocity(expectedVelocity);
		this.priority = priority;
		this.setCounter(counter);
		this.setTimestamp(timestamp);
		this.setLocation(location);
		this.setPolarity(polarity);
	}
	
	public void setSignalTier(int tier)
	{
		this.tier = tier;
	}
	
	public int getSignalTier()
	{
		return this.tier;
	}

	public int getLabel() {
		return label;
	}

	public void setLabel(int label) {
		this.label = label;
	}

	public int getPolarity() {
		return polarity;
	}

	public void setPolarity(int polarity) {
		this.polarity = polarity;
	}

	public int getExpectedArrivalTime() {
		return expectedArrivalTime;
	}

	public void setExpectedArrivalTime(int expectedArrivalTime) {
		this.expectedArrivalTime = expectedArrivalTime;
	}

	public float getExpectedVelocity() {
		return expectedVelocity;
	}

	public void setExpectedVelocity(float expectedVelocity) {
		this.expectedVelocity = expectedVelocity;
	}

	public int getPriority() {
		return priority;
	}

	public void increasePriority(int value) {
		this.priority += value;
	}

	/**
	 * @return the counter
	 */
	public int getCounter() {
		return counter;
	}

	/**
	 * @param counter the counter to set
	 */
	public void setCounter(int counter) {
		this.counter = counter;
	}
	
	public void increaseCounter()
	{
		counter++;
	}

	/**
	 * @return the timestamp
	 */
	public int getTimestamp() {
		return timestamp;
	}

	/**
	 * @param timestamp the timestamp to set
	 */
	public void setTimestamp(int timestamp) {
		this.timestamp = timestamp;
	}

	/**
	 * @return the location
	 */
	public Point2D.Float getLocation() {
		return location;
	}

	/**
	 * @param location the location to set
	 */
	public void setLocation(Point2D.Float location) {
		this.location = location;
	}

	
	public void setPriority(int priority) {
		this.priority = priority;
		
	}
} 