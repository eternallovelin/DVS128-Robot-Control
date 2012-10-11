package jaer.myjaer.neuron;

import jaer.myjaer.robotcontrol.AbstractImageReceiver.FilterLevel;
import jaer.myjaer.robotcontrol.Data;
import jaer.myjaer.robotcontrol.ImageReceiver;

import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Float;
import java.io.IOException;
import java.util.HashMap;
import java.util.LinkedList;

/** This is the neuron's memory specification
 * Each neuron has two separate memory banks: for high and low signal channels
 * Low channel: signals from preceding neuron in chain
 * High channel: signals from next neuron in chain
 */
public class Memory implements Cloneable
{
	private HashMap<Integer,Signal> signals;
	private LinkedList<Integer> signalLabels;
	private LinkedList<Integer> scheduled;
	
	private int lastFired;
	private int index;
	private Neuron neuron;
	
	private static ImageReceiver filter;
	
    /**
    * Constructor
    * @param index
    * @param neuron the Neuron object
    */
	public Memory(int index, Neuron neuron)
	{
		setSignals(new HashMap<Integer,Signal>());
		setSignalLabels(new LinkedList<Integer>());
		setScheduled(new LinkedList<Integer>());
		this.index = index;
		this.neuron = neuron;
		eraseMemory();
	}

	/**
    * Confirms a signal and calculates movement vector
    * @param signal The signal
    */
	public void confirm(Signal signal) {
		signal.setPriority(6);
		signal.setSignalTier(index-1); // Am only interested in a signal's cell number when it's confirmed
		Point2D.Float neuronLocation = neuron.getLocation();
		Point2D.Float signalLocation = signal.getLocation();
		Point2D.Float vector = new Point2D.Float(neuronLocation.x - signalLocation.x, neuronLocation.y - signalLocation.y);
		signal.setLocation(neuronLocation);
		if(checkVector(neuronLocation, vector) && checkSignalLocation(neuronLocation))
		{
			Data dat = filter.getDataInstance();
			if(dat != null)
			{
				dat.addSignal(signal);
				if(filter.LEVEL == FilterLevel.EVALUATION)
					filter.increaseConfirmedSignalCounter();
				
			}
			filter.getOpticalFlowVectors().put(signal.getLocation(), new Point2D.Float(signal.getLocation().x + vector.x,signal.getLocation().y+vector.y));
			filter.increaseSignalCounter();
		}		
	}
	
    /**
    * Checks if the signal is within the established image boundaries used for processing 
    * @param location: the location of the signal
    */
	public boolean checkSignalLocation(Point2D.Float location) {
		if(location.x == 64 || location.y == 64)
			return false;
		//Point2D.Float signalVector = (location.x < 64) ?
				//new Point2D.Float(location.x, location.y - 64) :
				//new Point2D.Float(location.x - 128, location.y - 64);
		Point2D.Float signalVector = new Point2D.Float(location.x - 64, location.y - 64);		
		float semiMinorAxis = filter.getSemiMinorAxis();
		float semiMajorAxis = 64f;
		int middleCRadius = 5;
		Point2D.Float vectorToImageCenter = new Point2D.Float(location.x - 64, location.y - 64);
		float alignedPosition = ((signalVector.x*signalVector.x) / (semiMajorAxis * semiMajorAxis)) + 
								((signalVector.y*signalVector.y) / (semiMinorAxis*semiMinorAxis));
		boolean withinSemiEllipse = (alignedPosition <= 1) ?
				true : 
				false;
		boolean withinMiddleCircle = (middleCRadius*middleCRadius >= (vectorToImageCenter.x * vectorToImageCenter.x) + 
																	 (vectorToImageCenter.y*vectorToImageCenter.y)) ?
																 	 true :
																	 false;
		return (withinSemiEllipse && !withinMiddleCircle) ? 
				true : 
				false;
	}

	/**
     * Checks whether the signal is moving in the right direction
     * @param location The signal location
	 * @param vector The movement vector
	 */
	public boolean checkVector(Point2D.Float location, Point2D.Float vector)
	{
		Point2D.Float target = getTargetVector(location);
		return ( vector.x / target.x < 0 || vector.y / target.y < 0 )? false : true;
	}

    /** 
     * Checks the memory for signal matches
     * @param now The current timestamp
     * @return a matched signal or null
     */
	public Signal checkForMatch(int now) {
		int bestTime = 100000000;
		Signal bestMatch = null;
		for(int hash: getSignalLabels())
		{
			Signal signal = getSignals().get(hash);
			//numberSignals.add(signal.getLabel());
			int polarity = neuron.computePolarity();
			int time = Math.abs(now - signal.getExpectedArrivalTime());
			int thresh = filter.getArrivalThreshold();
			if(-thresh <= time && time <= thresh && signal.getPolarity() == polarity)
			{
				if(time < bestTime)
				{
					if(filter.LEVEL == FilterLevel.DEBUG)
					{
						System.out.println("MATCH: NEURON:"+neuron.getCellNumber()+".Signal is: "+signal.getLabel()+
						"(arrival: "+ signal .getExpectedArrivalTime()+", velocity: "+signal.getExpectedVelocity()+"). Time difference: " + time);
					}
					bestTime = time;
					bestMatch = signal;
				}				
			}
		}
		if(bestMatch != null)
		{
			if(filter.LEVEL == FilterLevel.DEBUG)
			{
				System.out.println("BEST MATCH: NEURON:"+neuron.getCellNumber()+".Signal is: "+bestMatch.getLabel()+
						"(arrival: "+ bestMatch.getExpectedArrivalTime()+", velocity: "+bestMatch.getExpectedVelocity()+")");
			}
			schedule_for_removal(bestMatch.getLabel());
		}
			
		return bestMatch;
		
	}
	
	/** 
	 * To avoid concurrentModificationExceptions
	 * @param signal_hash
	 */
	public void schedule_for_removal(int signal_hash)
	{
		if(!getScheduled().contains(signal_hash))
			getScheduled().add(signal_hash);
	}
	
	/** 
	 * To avoid concurrentModificationExceptions
	 * @param signal_hash
	 */
	public void doSignalRemoval()
	{
		for(int hash: getScheduled())
		{
			removeSignal(hash);
			
		}
		getScheduled().clear();
		
	}

    /**
    * Remove the signal from memory
    * @param signal_hash 
    */
	public void removeSignal(int signal_hash) {
		int before = getSignalLabels().indexOf(signal_hash);
		getSignals().remove(signal_hash);
		int hash_index = getSignalLabels().indexOf(signal_hash);
		if(hash_index == -1)
			return;
		try
		{
			getSignalLabels().remove(hash_index);
		}
		catch(IndexOutOfBoundsException e)
		{
		}
	}

    /**
    * Wipe the memory
    *
    */
	void eraseMemory()
	{
		if(!getSignals().isEmpty())
			getSignals().clear();
		if(!getSignalLabels().isEmpty())
			getSignalLabels().clear();
		lastFired = 0;
		if(filter.LEVEL == FilterLevel.DEBUG)
		{
			String signals = "";
			for(int label : this.signalLabels)
			{
				signals += label+";";
			}
			System.out.println("ERASE: NEURON:"+neuron.getCellNumber()+". Value of isEmpty():"+is_memory_empty()+"; Signals:"+signals);
		}
	}
	
    /**
    * Checks whether the memory contains any signals
    */
	boolean is_memory_empty()
	{
		return (getSignals().isEmpty()) ? true : false;
	}
	
    /**
    * copies signals from one memory instance to another
    */
	void copyFromMemory(Memory memory) throws CloneNotSupportedException
	{
		HashMap<Integer, Signal> signal_copy = (HashMap<Integer, Signal>) memory.getSignals().clone();
		LinkedList<Integer> hash_copy = (LinkedList<Integer>) memory.getSignalLabels().clone();
		String signals = "";
		for(int i : hash_copy)
		{
			if(!getSignalLabels().contains(i))
			{
				if(filter.LEVEL == FilterLevel.DEBUG)
					signals += i+";";
				this.getSignals().put(i, signal_copy.get(i));
    			this.getSignalLabels().add(i);
			}        			
		}
		if(filter.LEVEL == FilterLevel.DEBUG)
		{
			System.out.println("COPY: NEURON:"+neuron.getCellNumber()+". Signals:" + signals);
		}
		lastFired = memory.lastFired;
	}	
	 

    protected Point2D.Float getTargetVector(Point2D.Float vector)
    {
    	return new Point2D.Float(vector.x - 64, vector.y - 64);
    }

	public LinkedList<Integer> getSignalLabels() {
		return signalLabels;
	}

	public void setSignalLabels(LinkedList<Integer> signalLabels) {
		this.signalLabels = signalLabels;
	}

	public HashMap<Integer,Signal> getSignals() {
		return signals;
	}

	public void setSignals(HashMap<Integer,Signal> signals) {
		this.signals = signals;
	}

	public LinkedList<Integer> getScheduled() {
		return scheduled;
	}

	public void setScheduled(LinkedList<Integer> scheduled) {
		this.scheduled = scheduled;
	}
	
	public static void setFilter(
			ImageReceiver filter) {
		if(filter instanceof ImageReceiver)
			Memory.filter = (ImageReceiver) filter;
	}
	
	public static ImageReceiver getFilter()
	{
		return filter;
	}
    
	
}  
