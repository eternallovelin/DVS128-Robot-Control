package jaer.myjaer.neuron;

import java.awt.geom.Point2D;

import jaer.myjaer.robotcontrol.AbstractImageReceiver;
import jaer.myjaer.robotcontrol.AbstractImageReceiver.FilterLevel;
import jaer.myjaer.robotcontrol.ImageReceiver4;

/**
 * A memory bank contains both real and temporary memories. For easier access
 * @author vanxa
 *
 */
public class MemoryBank
{
	int id; /** 0-low,1-high */
	
	/**
	 * Default angle between two radii
	 */
	Memory realMemory;
	
	/**
	 * Default angle between two radii
	 */
	Memory tempMemory;
	
	/**
	 * Default angle between two radii
	 */
	int index;
	
	
	/**
	 * Default angle between two radii
	 */
	boolean to_copy;
	
	
	/**
	 * Default angle between two radii
	 */
	private Neuron neuron;
	
	/**
	 * Default angle between two radii
	 */
	private int lastSignalPolarity = -1;
	
	/** Records the last time the signal polarity has been switched */
	private int lastTimeSwitched = 0;
	
	/**
	 * Default angle between two radii
	 */
	private static AbstractImageReceiver filter;
	
	/**
	 * Default angle between two radii
	 */
	public MemoryBank(int id, int index, Neuron neuron)
	{
		this.index = index;
		this.id = id;
		realMemory = new Memory(index, neuron);
		tempMemory = new Memory(index, neuron);
		to_copy=false;
		this.neuron = neuron;
	}
	
	/**
	 * Default angle between two radii
	 */
	public void scheduleForCopy(boolean to_copy) {
		this.to_copy = to_copy;
	}
	
	/**
	 * Default angle between two radii
	 */
	public boolean isScheduledForCopy()
	{
		return to_copy;
	}

	/**
	 * Default angle between two radii
	 */
	public void reset(boolean isHardReset)
	{
		if(isHardReset)
		{
			realMemory.eraseMemory();
			tempMemory.eraseMemory();
			to_copy = false;

		}
		lastSignalPolarity = -1;
		lastTimeSwitched = 0;
	}
	
	
	/**
     * Update memory when not firing
     * @param instance
     */
    synchronized public void silentUpdateMemory(int time)
    {
    	for(int index : realMemory.getSignalLabels())
       	{
       		Signal signal= realMemory.getSignals().get(index);
       		//numberSignals.add(signal.getLabel());
       		if(time - signal.getExpectedArrivalTime() > filter.getArrivalThreshold())
       		{
       			signal.increasePriority(-1);
       			if(filter.LEVEL == FilterLevel.DEBUG)
				{
					System.out.println("SILENTUPDATE:"+"NEURON:"+neuron.getCellNumber()+".Reducing priority of signal "+signal.getLabel()+
							";Time:"+time+";expArrTime:"+signal.getExpectedArrivalTime()+";expVelocity:"+
							signal.getExpectedVelocity()+";Priority is now:"+signal.getPriority());
				}
       		}
       		if(signal.getPriority() <= filter.getPriorityThreshold())
       		{
       			realMemory.schedule_for_removal(signal.getLabel());
       			if(filter.LEVEL == FilterLevel.DEBUG)
				{
					System.out.println("SILENTUPDATE:"+"NEURON:"+neuron.getCellNumber()+".Signal "+ signal.getLabel()+" scheduled for removal");
				}
       			//if(numberSignals.contains(signal.getLabel()))
       				//numberSignals.remove(signal.getLabel());
       		}
       	}
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
	 * It is best to reduce the number of signals: only send a signal if last signal had different (opposite) polarity
	 * Also, because this flag may become stale (a neuron doesn't transmit any signals because it has transmitted a signal with same polarity,
	 * but a very long time ago), a time threshold is set
	 * This condition is also useful to reduce wrongly matched signals: neurons that fire consecutively 
	 * from a single stimulus should not produce multiple signals
	 */
    public boolean checkPolarityCondition(int polarity, int now)
    {
    	return polarity != lastSignalPolarity || (now - lastTimeSwitched) > filter.getSignalPolarityResetThreshold();
    }
    
    /**
     * Update single memory bank when firing
     * @param tempMemoryory 
     * @param memory 
     */
    synchronized public void updateSingleMemory(int now) 
    {
    	Point2D.Float edge = neuron.computeAndReturnEdgePosition();
    	if(edge == null) return;
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
        	MemoryBank neighbor_memory = null;
        	if(neighbor != null)
        	{
        		if(id == 1)
            	{
            		neighbor_memory = neighbor.getLowMemory();        		
            	}
            		
            	else if(id == 0)
            		neighbor_memory = neighbor.getHighMemory();
        	}    	
        	Signal match = realMemory.checkForMatch(now);
        	if(match == null || realMemory.is_memory_empty())
        	{        		
        		Signal curr_signal = createNewSignal(this, 1, 1, polarity, now);
        		propagate(neighbor_memory, curr_signal);
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
    			propagate(neighbor_memory, match);
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
	 * Default angle between two radii
	 */
    Signal createNewSignal(MemoryBank bank_mem, int counter, int signal_counter,int polarity, int now) {
    	float[] distances = neuron.getDistances(bank_mem.id); 
		float expectedVelocity = filter.getExpectedInitialSignalVelocity();
		float travelTime = (distances[0] / expectedVelocity) * 1000000f;
		int expectedArrivalTime = now + (int)travelTime;
		Signal signal = new Signal(expectedArrivalTime, expectedVelocity, 4, signal_counter, counter, neuron.getLastEventTimestamp(), neuron.getLocation(), polarity);
		//signal.setLabel(Neuron.increaseAndReturnSignalCounter()); // Check for clashes here!
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
	 * Default angle between two radii
	 */
    public Memory getRealMemory()
    {
    	return realMemory;
    }
    
    /**
	 * Default angle between two radii
	 */
    public Memory getTempMemory()
    {
    	return tempMemory;
    }
    
    /**
	 * Default angle between two radii
	 */
    void propagate(MemoryBank neighborMemory, Signal signal)
    {
    	if(neighborMemory == null)
    		return;
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
	 * Default angle between two radii
	 */
    public static void setFilter(
    		AbstractImageReceiver filter) {
		if(filter instanceof ImageReceiver4)
			MemoryBank.filter = (ImageReceiver4) filter;
	}
	
    /**
	 * Default angle between two radii
	 */
	public static AbstractImageReceiver getFilter()
	{
		return filter;
	}
}