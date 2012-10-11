package jaer.myjaer.neuron;

import java.awt.geom.Point2D;

/**
* This is the signal object. It stores information about the passing stimulus, as 
* detected by the associated neuron object. Such information is relayed between neurons in 
* order to track the movement of the stimulus. 
*/
public class Signal
{
	private int label; // Signal label, used as a key to hash table storing all signals in memory
	private int polarity; // Stimulus polarity, either Polarity.On or Polarity.Off
	private int expectedArrivalTime; // The estimated arrival time of the stimulus at a neighboring neuron
	private float expectedVelocity;  // The estimated velocity of the stimulus
	private int priority; // All signals below a global priority threshold are wiped from memory. The priority determines how long a signal is stored
	private int counter; // The number of hops from one neuron to another
	private int timestamp; // Time of creation/detection
	private Point2D.Float location; // Location of the signal, corresponds to the location of the detecting neuron
	private	int tier; // The neuron's radial tier
	
    /**
    * Constructor
    * @param expectedArrivalTime
    * @param expectedVelocity
    * @param priority
    * @param counter
    * @param timestamp
    * @param location
    * @param polarity
    */
	public Signal(int expectedArrivalTime, float expectedVelocity, int priority, int counter, int timestamp, Point2D.Float location, int polarity)
	{
		this.setExpectedArrivalTime(expectedArrivalTime);
		this.setExpectedVelocity(expectedVelocity);
		this.priority = priority;
		this.setCounter(counter);
		this.setTimestamp(timestamp);
		this.setLocation(location);
		this.setPolarity(polarity);
	}
	
    /**
    * @param tier The neuron tier
    */
	public void setSignalTier(int tier)
	{
		this.tier = tier;
	}
	
    /**
    * @return tier
    */
	public int getSignalTier()
	{
		return this.tier;
	}

    /**
    * @return label 
    */
	public int getLabel() {
		return label;
	}

    /**
    * @param label The signal's label
    */
	public void setLabel(int label) {
		this.label = label;
	}

    /**
    * @return polarity
    */
	public int getPolarity() {
		return polarity;
	}

    /**
    * @param polarity the polarity of the stimulus
    */
	public void setPolarity(int polarity) {
		this.polarity = polarity;
	}

    /**
    * @return expectedArrivalTime
    */
	public int getExpectedArrivalTime() {
		return expectedArrivalTime;
	}

    /**
    * @param expectedArrivalTime the estimated signal arrival time
    */
	public void setExpectedArrivalTime(int expectedArrivalTime) {
		this.expectedArrivalTime = expectedArrivalTime;
	}

    /**
    * @return expectedVelocity
    */
	public float getExpectedVelocity() {
		return expectedVelocity;
	}

    /**
    * @param expectedVelocity the estimated velocity of the signal
    */
	public void setExpectedVelocity(float expectedVelocity) {
		this.expectedVelocity = expectedVelocity;
	}

    /**
    * @return priorty
    */
	public int getPriority() {
		return priority;
	}

	public void increasePriority(int value) {
		this.priority += value;
	}

	/**
	 * @return counter
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

	
    /**
    * @param priority 
    */
	public void setPriority(int priority) {
		this.priority = priority;
		
	}
} 
