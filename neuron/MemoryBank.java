package jaer.myjaer.neuron;

import java.awt.geom.Point2D;
import jaer.myjaer.robotcontrol.AbstractImageReceiver;
import jaer.myjaer.robotcontrol.AbstractImageReceiver.FilterLevel;
import jaer.myjaer.robotcontrol.ImageReceiver;

/**
 * A memory bank is used to store signals within the neuron object.
 * It contains both real and temporary memory stores, where the temporary memory
 * is wiped when a processing iteration terminates. 
 * @author Ivan Konstantinov
 *
 */
public class MemoryBank
{
	int id; /** 0-low,1-high */
	
	/**
	 * The real memory instance
	 */
	Memory realMemory;
	
	/**
	 * The temporary memory instance
	 */
	Memory tempMemory;
	
	/**
	 * The index of the neuron
	 */
	int neuronIndex;
	
	
	/**
	 * This is switched to true if the contents of the temporary memory
     * instance must be copied into the real memory
	 */
	boolean shouldCopy;
	
	
	/**
	 * The neuron object
	 */
	private Neuron neuron;
	
	/**
	 * The polarity bit of the last stored signal
	 */
	private int lastSignalPolarity = -1;
	
	/** Records the last time the signal polarity has been switched */
	private int lastTimeSwitched = 0;
	
	/**
	 * The image processor instance must extend the AbstractImageReceiver class
	 */
	private static AbstractImageReceiver filter;
	
	/**
	 * Constructor
     * @param id: The id of the the memory bank
     * @param neuron: The neuron object
	 */
	public MemoryBank(int id, Neuron neuron)
	{
		this.id = id;
		this.neuron = neuron;
        neuronIndex = neuron.getIndex();
		realMemory = new Memory(neuronIndex, neuron);
		tempMemory = new Memory(neuronIndex, neuron);
		shouldCopy=false;
	}
	
	/**
	 * Schedules the bank's temporary memory contents to be copied into the
     * real memory instance.
     * @param shouldCopy: Boolean value determining whether the data transfer should occur
	 */
	public void scheduleForCopy(boolean shouldCopy) {
		this.shouldCopy = shouldCopy;
	}
	
	/**
	 * Returns shouldCopy
     * @return shouldCopy: the boolean value which determines whether data transfer
     * between the temporary and real memory instances should occur
	 */
	public boolean isScheduledForCopy()
	{
		return shouldCopy;
	}

	/**
	 * Resets the memory bank. There are two reset modes: hard and soft.
     * A soft reset will only reset variables which are related to the firing
     * behavior of the neuron. A hard reset will remove all data
     * @param isHardReset: the type of reset action to be taken
	 */
	public void reset(boolean isHardReset)
	{
		if(isHardReset)
		{
			realMemory.eraseMemory();
			tempMemory.eraseMemory();
			shouldCopy = false;

		}
		lastSignalPolarity = -1;
		lastTimeSwitched = 0;
	}
	
	
	/**
     * Update memory data when the neuron is not firing
     * @param time: The time of update
     */
    synchronized public void silentUpdateMemory(int time)
    {
    	for(int index : realMemory.getSignalLabels())
       	{
       		Signal signal= realMemory.getSignals().get(index);
       		if(time - signal.getExpectedArrivalTime() > filter.getArrivalThreshold())
       		{
       			signal.increasePriority(-1);
       			if(filter.LEVEL == FilterLevel.DEBUG)
				{
                    // Debug mode is ON
					System.out.println("SILENTUPDATE:"+"NEURON:"+neuron.getCellNumber()+".Reducing priority of signal "+signal.getLabel()+
							";Time:"+time+";expArrTime:"+signal.getExpectedArrivalTime()+";expVelocity:"+
							signal.getExpectedVelocity()+";Priority is now:"+signal.getPriority());
				}
       		}
       		if(signal.getPriority() <= filter.getPriorityThreshold())
       		{
       			realMemory.scheduleForRemoval(signal.getLabel());
       			if(filter.LEVEL == FilterLevel.DEBUG)
				{
					System.out.println("SILENTUPDATE:"+"NEURON:"+neuron.getCellNumber()+".Signal "+ signal.getLabel()+" scheduled for removal");
				}
       		}
       	}
        // Actual signal removal occurs here in order to avoid ConcurrentModificationException errors
		if(!realMemory.getScheduled().isEmpty())
		{
			try
			{
				realMemory.doSignalRemoval();
			}
			catch(IndexOutOfBoundsException e)
			{
				e.printStackTrace();
			}
		}
    }
    
    /**
    * Checks whether the current stimulus polarity bit is the same as the last polarity bit that was computed.
    * @param polarity: The current stimulus' polarity
    * @param now: The current time of checking 
    * @param return Boolean value, determining whether the current signal is a repeat (too close and identical) to the last detected signal
    *
    */
    public boolean checkPolarityCondition(int polarity, int now)
    {
        /**
         * It is best to reduce the number of signals: only send a signal if last signal had different (opposite) polarity
         * Also, because this flag may become stale (a neuron doesn't transmit any signals because it has transmitted a signal with same polarity,
         * but a very long time ago), a time threshold is set
         * This condition is also useful to reduce wrongly matched signals: neurons that fire consecutively 
         * from a single stimulus should not produce multiple signals
         */
    	return polarity != lastSignalPolarity || (now - lastTimeSwitched) > filter.getSignalPolarityResetThreshold();
    }
    
    /**
     * Update a single memory bank when neuron has fired
     * @param now: The time of update (current time)
     */
    synchronized public void updateSingleMemory(int now) 
    {
    	Point2D.Float edge = neuron.computeAndReturnEdgePosition(); // This is the (x,y) position of the currently detected stimulus
    	if(edge == null) // If no edge detected, don't continue
        {
            return;
        }
    	Point2D.Float location = neuron.getLocation();
    	Point2D.Float vector = new Point2D.Float(edge.x-location.x, edge.y-location.y);
    	float distance = (float) Math.sqrt(vector.x*vector.x + vector.y*vector.y);
    	int polarity = neuron.computePolarity();
    	boolean polarityCheck = checkPolarityCondition(polarity, now);
    	if(distance <= 4 && polarityCheck)
    	{
    		if(filter.LEVEL == FilterLevel.DEBUG)
    			System.out.println("UPDATESINGLE (CREATE/MATCH): NEURON: "+ neuron.getCellNumber()+". Distance between neuron and edge is "+ distance+".");
    		Neuron neighbor = neuron.fetchNeighbor(id);
        	MemoryBank neighborMemory = null;
        	if(neighbor != null)
        	{
        		if(id == 1)
            	{
            		neighborMemory = neighbor.getLowMemory();        		
            	}
            		
            	else if(id == 0)
            		neighborMemory = neighbor.getHighMemory();
        	}    	
        	Signal match = realMemory.checkForMatch(now);
        	if(match == null || realMemory.isMemoryEmpty())
        	{        		
        		Signal currSignal = createNewSignal(id, polarity, now);
        		propagate(neighborMemory, currSignal);
        		lastSignalPolarity = polarity;
        		lastTimeSwitched = now;
        		
        	}
        	else
        	{
    			match.increaseCounter();
    			float[] distances = neuron.getDistances(id);
    			float time = (float)((now - match.getTimestamp())/1000000f);
    			if(time > 0)
    				match.setExpectedVelocity(distances[1] / time);
    			match.setTimestamp(now);
    			float travelTime = (distances[0] / match.getExpectedVelocity())*1000000f;
    			match.setExpectedArrivalTime(now + (int)travelTime);
    			if(match.getCounter() >= filter.getConfirmThreshold())
    			{
    				realMemory.confirm(match);
    				if(filter.LEVEL == FilterLevel.DEBUG)
    				{
    					System.out.println("UPDATESINGLEMEMORY: NEURON:"+"NEURON:"+neuron.getCellNumber()+"Confirming signal "+match.getLabel()+".Time is:"+now);
    				}
    			}
    			match.setLocation(neuron.getLocation());	
    			realMemory.doSignalRemoval();
    			propagate(neighborMemory, match);
    			lastSignalPolarity = neuron.computePolarity();
    			lastTimeSwitched = now;
    		}
    	}
    	else
    	{
    		if(filter.LEVEL == FilterLevel.DEBUG)
    		{
    			if(polarityCheck)
    				System.out.println("UPDATESINGLE (DELAY): NEURON: "+ neuron.getCellNumber()+". Distance between neuron and edge is "+ distance+". Delaying signal");
        		else
        			System.out.println("UPDATESINGLE (IGNORE): NEURON: "+ neuron.getCellNumber()+". Polarity check returned false. Ignoring spike.");
    		}    			
    		return;
    	}

	}
    
    /**
     * Creates a new signal and returns it
     * @param id: The id of the memory bank 
     * @param polarity: The polarity bit of the corresponding stimulus
     * @param now: The timestamp of the signal (current time)
	 */
    Signal createNewSignal(int id,int polarity, int now) {
    	float[] distances = neuron.getDistances(id); 
		float expectedVelocity = filter.getExpectedInitialSignalVelocity();
		float travelTime = (distances[0] / expectedVelocity) * 1000000f;
		int expectedArrivalTime = now + (int)travelTime;
		Signal signal = new Signal(expectedArrivalTime, expectedVelocity, 4, 1, neuron.getLastEventTimestamp(), neuron.getLocation(), polarity);
		signal.setLabel(signal.hashCode());
		if(filter.LEVEL == FilterLevel.DEBUG)
		{
			System.out.println("CREATE: NEURON:"+neuron.getCellNumber()+".Creating new signal.Time:"+now+";ExpVelocity:"+expectedVelocity+";expArrivalTime:"+expectedArrivalTime+";Polarity:"+polarity+";Label:"+signal.getLabel());
		}
		if(filter.LEVEL == FilterLevel.EVALUATION)
		{
			filter.increaseCreatedSignalsCounter();
		}
		return signal;
	}
    
    /**
     * Returns the real memory instance
     * @return realMemory: the real memory instance
	 */
    public Memory getRealMemory()
    {
    	return realMemory;
    }
    
    /**
     * Returns the temporary memory instance
     * @return tempMemory: the temporary memory instance
	 */
    public Memory getTempMemory()
    {
    	return tempMemory;
    }
    
    /**
	 * Propagate the signal along the neural radius, upwards (away from the nodal point), or downwards (towards the nodal
     * point).
     * @param neighborMemory: the neighboring memorym to which the signal is propagated
     * @param signal: the signal to be propagated 
	 */
    void propagate(MemoryBank neighborMemory, Signal signal)
    {
    	if(neighborMemory == null)
        {    
    		return; 
        }
    	Neuron neighbor = neuron.fetchNeighbor(id);
    	if(filter.LEVEL == FilterLevel.DEBUG)
    		System.out.println("PROPAGATE: "+"NEURON:"+neuron.getCellNumber()+".Propagating signal " + signal.getLabel() + 
    				" to neuron " + neighbor.getCellNumber()+". Expected arrival time: " + signal.getExpectedArrivalTime()+
    				". Velocity: "+ signal.getExpectedVelocity());
    	signal.setSignalTier(index);
    	int hash = signal.getLabel();
    	neighborMemory.tempMemory.getSignals().put(hash, signal);
    	neighborMemory.tempMemory.getSignalLabels().add(hash);
    	neighborMemory.scheduleForCopy(true);
		filter.getMemoryUpdateList().add(neighbor);
    }
     
    /**
     * Setter for the Image Processor object
     * @param AbstractImageReceiver : the image processor object 
	 */
    public static void setFilter(AbstractImageReceiver filter) {
		if(filter instanceof ImageReceiver)
			MemoryBank.filter = (ImageReceiver) filter;
	}
	
    /**
     * Getter for the Image Processor object
     * @return AbstractImageReceiver: The image processor must be an instance of an AbstractImageReceiver
	 */
	public static AbstractImageReceiver getFilter()
	{
		return filter;
	}
}
