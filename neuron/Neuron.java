package jaer.myjaer.neuron;

import java.awt.geom.Point2D;
import java.util.LinkedList;

import net.sf.jaer.event.PolarityEvent;
import net.sf.jaer.event.PolarityEvent.Polarity;

import jaer.myjaer.robotcontrol.AbstractImageReceiver;
import jaer.myjaer.robotcontrol.AbstractImageReceiver.FilterLevel;
import jaer.myjaer.robotcontrol.ImageReceiver4;

/**
     * Definition of leaky integrate and fire (LIF) neuron.
     * The receptive field is a partial area of events-occuring space.
     * Events within the receptive field of a neuron are considered strongly correalated.
     * Spacing of the receptive field of two adjacent LIF neurons is decided to the half of side length of the receptive field to increase the spatial resolution.
     * Thus, each neuron shares half area of the receptive field with its neighbor.
     */
public class Neuron {

	/**
	 * AccumulationTime variable
	 * @deprecated
	 */
	@Deprecated
	private int startAccumulationTime;
	
	/**
	 * The radius index on which the neuron is located
	 */
	private int radiusNum;
	
	/**
	 * Number of On events that comprise an ON edge
	 */
	private int onEdge;
	
	/**
	 * Number of On events that comprise an OFF edge
	 */
	private int offEdge;
	@Deprecated
	private static final int COUNTER_WRAP = 1000;
		    	
	/** Low and high refer to neuron position on the radius */
	private Neuron lowNeuron,highNeuron;
	
    /**
     * Defined as neuron position on a radius, 0 < index < neuronsOnRadius
     */
    private int index;
    
     /**
     *  spatial location of a neuron in chip pixels
     */
    private Point2D.Float location = new Point2D.Float();
    
    /**
     * true if the neuron fired a spike.
     */
    private boolean fired = false;

    /** The "membranePotential" of the neuron.
     * The membranePotential decays over time (i.e., leaky) and is incremented by one by each collected event.
     * The membranePotential decays with a first order time constant of tauMP in us.
     * The membranePotential dreases by the amount of MPJumpAfterFiring after firing an event.
     */
    private float membranePotential = 0;

    /**
     *  number of firing neighbors
     */
    private int numFiringNeighbors = 0;

    /**
     * This is the last in timestamp ticks that the neuron was updated, by an event
     */
    private int lastEventTimestamp;

    /**
     * Defined as radius*(neuronsOnRadius-1) + num_neuron-1
     */
    private int cellNumber;

     /**
     * size of the receptive field.
     */
    private int receptiveFieldSize;

    
    /**
	 * The Image receiver
	 */
    private static AbstractImageReceiver filter;

	private static int RADIUS;
    
	/** Each neuron tracks the position of the stimulus within its receptive field (similar to direction-selective neurons in terms 
	 * of their local computations of motion. A signal is created only when the edge is within immediate proximity to the neuron; thus propagation of 
	 * a signal is delayed. 
	 */
    private Point2D.Float edgePosition;
    /** Used to compute the average edge position */
    private int edgeEventNormCounter;
    /**
     *  Temporary memories are used to store memory data without affecting the processing logic for the current iteration
     *  For example, updateNeurons is called and Neuron A sets values in Neuron B's memory instance, then Neuron B is processed and it sees the 
     *  new data and reacts to it, rather than react during the next processing iteration. 
     */
    private MemoryBank lowMemory, highMemory;
	private LinkedList<Neuron> rNeurons;
    
     /**
     * Construct an LIF neuron with index.
     * @param rNeurons 
     *
     * @param indexX
     * @param indexY
     */
    public Neuron(int cellNumber, LinkedList<Neuron> rNeurons, int index, Point2D.Float location, int radius) {
        this.cellNumber = cellNumber;
        this.index = index;
        this.location.x = location.x;
        this.location.y = location.y;
        this.rNeurons = rNeurons;
        this.setRadiusNum(radius);
        resetEdgeCounters();
        edgePosition = new Point2D.Float();
        edgeEventNormCounter = 0;
                
        setHighMemory(new MemoryBank(1, index, this));
        setLowMemory(new MemoryBank(0, index, this));        
        reset(true);
    }
    
    /**
     * Resets the edge position
     */
    public void resetEdgePosition()
    {
    	edgeEventNormCounter = 0;
    	edgePosition = new Point2D.Float();
    }
    

    public void resetEdgeCounters()
    {
    	onEdge = 0;
    	offEdge = 0;
    }
   
    /**
    * Resets a neuron with initial values
    */
    public void reset(boolean isHardReset) {
        fired = false;
        membranePotential = 0;
        numFiringNeighbors = 0;
        lastEventTimestamp = 0;
		if(lowMemory != null)
			lowMemory.reset(isHardReset);
	    if(getHighMemory() != null)
	    	getHighMemory().reset(isHardReset);
        setStartAccumulationTime(0);
        onEdge = 0;
        offEdge = 0;
        resetEdgePosition();
    }
    
    @Override
    public int hashCode() {
        return cellNumber;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if ((obj == null) || (obj.getClass() != this.getClass())) {
            return false;
        }

        Neuron test = (Neuron) obj;
        return cellNumber == test.cellNumber;
    }
  
    /**
     * updates a neuron with an additional event.
     *
     * @param event
     * @param weight
     */
    public void addEvent(PolarityEvent event,float weight) {
    	incrementMP(event.getTimestamp(), weight);
        lastEventTimestamp = event.getTimestamp();
        if(getStartAccumulationTime() == 0)
        	setStartAccumulationTime(lastEventTimestamp);
        if(event.polarity==Polarity.On)
        {
        	onEdge++;
        	if(offEdge >= 3)
        		offEdge -= 3;
        	else
        		offEdge = 0;
        }        	
        else
        {
        	offEdge++;
        	if(onEdge > 3)
        		onEdge -= 3;
        	else
        		onEdge = 0;
        }
        edgePosition.x += event.x;
        edgePosition.y += event.y;
        edgeEventNormCounter++;
                
        if(filter.LEVEL == FilterLevel.DEBUG)
        {
        //	System.out.println("neuron.addEvent(): Neuron:" + cellNumber+"; Time:"+lastEventTimestamp);
        }
    }

	/**
     * Computes and returns membranePotential at time t, using the last time an event hit this neuron
     * and the tauMP. Does not change the membranePotential itself.
     *
     * @param t timestamp now.
     * @return the membranePotential.
     */
    public float getMPNow(int t) {
        float m = membranePotential * (float) Math.exp(((float) (lastEventTimestamp - t)) / filter.getMPTimeConstantUs());
        return m;
    }

    /**
     * returns the membranePotential without considering the current time.
     *
     * @return membranePotential
     */
    public float getMP() {
        return membranePotential;
    }

    /**
     * sets membranePotential
     * @param membranePotential
     */
    public void setMP(float membranePotential){
        this.membranePotential = membranePotential;
    }

    /**
     * Increments membranePotential of the neuron by amount of weight after decaying it away since the lastEventTimestamp according
     * to exponential decay with time constant tauMP.
     *
     * @param timeStamp
     * @param weight
     */
    public void incrementMP(int timeStamp, float weight) {
        membranePotential = weight + membranePotential * (float) Math.exp(((float) lastEventTimestamp - timeStamp) / filter.getMPTimeConstantUs());
        if(membranePotential < -1.0f)
            membranePotential = -1.0f;
    }

    /**
     * returns the neuron's location in pixels.
     *
     * @return
     */
    final public Point2D.Float getLocation() {
        return location;
    }

    /**
     * returns true if the neuron fired a spike.
     * Otherwise, returns false.
     *
     * @return
     */
    final public boolean isFired() {
        return fired;
    }

    /**
     * checks if the neuron's membrane potential is above the threshold
     *
     * @return
     */
    public boolean isAboveThreshold(int lastTime) {
        if (getMPNow(lastTime) < filter.getMPThreshold()){
            fired = false;
        }else{
            // fires a spike
            fired = true;
            // decreases MP by MPJumpAfterFiring after firing
            reduceMPafterFiring();
            if(filter.LEVEL == FilterLevel.DEBUG)
            {
            	System.out.println("FIRE: "+"NEURON:"+cellNumber+".Time:"+lastTime+";Polarity:"+computePolarity());
            }
        }
        return fired;
    }
    
    
    /**
	 * Default angle between two radii
	 */
    public Point2D.Float getEdgePosition()
    {
    	return edgePosition;
    }

    /**
     * This function is used by the Memory Bank to track the position of the stimulus. 
     * It's a good idea to both compute the position and reset the variable values here.
     * @return pos
     */
    public Point2D.Float computeAndReturnEdgePosition()
    {
    	Point2D.Float pos = (edgeEventNormCounter != 0)?
    						new Point2D.Float(edgePosition.x/(float)(edgeEventNormCounter),edgePosition.y/(float)(edgeEventNormCounter)):
    						null;
    	
    	return pos;
    }
    
    /**
     * decreases MP by MPJumpAfterFiring after firing
     */
    public void reduceMPafterFiring(){
        membranePotential -= filter.getMPJumpAfterFiring();
    }

    @Override
    public String toString() {
        return String.format("LIF Neuron cell number=%d index=%d, location = %d, %d, membrane potential = %.2f",
                cellNumber,
                index,
                (int) location.x, (int) location.y,
                getMPNow(lastEventTimestamp));
    }


    /**
     * returns index
     *
     * @return
     */
    public int getIndex() {
        return index;
    }

    /**
     * returns the number of simutaneously firing neighbors
     *
     * @return
     */
    public int getNumFiringNeighbors() {
        return numFiringNeighbors;
    }

    /**
     * sets the number of simutaneously firing neighbors
     *
     * @param numFiringNeighbors
     */
    public void setNumFiringNeighbors(int numFiringNeighbors) {
        this.numFiringNeighbors = numFiringNeighbors;
    }

    /**
     * increases the number of firing neighbors
     *
     */
    public void increaseNumFiringNeighbors() {
        numFiringNeighbors++;
    }

    /**
     * returns the cell number of a neuron
     *
     * @return cell number
     */
    public int getCellNumber() {
        return cellNumber;
    }

    /**
     * returns the last event timestamp
     *
     * @return timestamp of the last event collected by the neuron
     */
    public int getLastEventTimestamp() {
        return lastEventTimestamp;
    }

    /**
     * sets the last event timestamp
     *
     * @param lastEventTimestamp
     */
    public void setLastEventTimestamp(int lastEventTimestamp) {
        this.lastEventTimestamp = lastEventTimestamp;
    }

    /** 
     * Sets the receptive field for the neuron
     * @param size the size of the field
     */
    public void setReceptiveFieldSize(int size)
    {
    	this.receptiveFieldSize = size;
    }
    
    /**
     * returns receptiveFieldSize
     *
     * @return
     */
    public int getReceptiveFieldSize() {
        return receptiveFieldSize;
    }
       
    /**
	 * Sets the lower neighbor
	 */
    public void setLowNeuron(Neuron n)
    {
    	this.lowNeuron = n;
    }
    
    /**
     * Gets the lower neighbor
     */
    public Neuron getLowNeuron()
    {
    	return lowNeuron;
    }
    
    /**
	 * Sets the higher neighbor
	 */
    public void setHighNeuron(Neuron n)
    {
    	highNeuron = n;
    }
    
    
    /**
     * Gets the higher neirghbor
     * @return
     */
    public Neuron getHighNeuron()
    {
    	return highNeuron;
    }
    
    
    /**
	 * Computes the distance between this neuron and its immediate neighbor, given by its index
	 */
    public float computeNeuronDistance(int _this, int neighbor)
    {
    	int numNeurons = rNeurons.size();
    	float distance;
    	if((_this == 1 && neighbor == 0) || (_this == numNeurons && neighbor == numNeurons + 1))
    		return 0;
    	else
    	{
    		if(_this == neighbor + 1)
    			distance = (float)(2*RADIUS*_this) / (float)(numNeurons*(numNeurons - 1));
    		else if(_this == neighbor - 1)
    			distance = (float)(2*RADIUS*neighbor) / (float)(numNeurons*(numNeurons-1));
    		else
    			distance = 0;
    		return distance;
    	}
    	       	
    }
    
    
    /**
     * 
     * @param id
     * @return
     */
    public Neuron fetchNeighbor(int id)
    {
    	if(id == 0) // Low neighbor
    		return lowNeuron;
    	else if(id == 1)
    		return highNeuron;
    	else
    		return null;
    }
    
    /**
	 * Checks the memory banks
	 */
    
    synchronized public void checkMemory(int now)
    {
    	if(fired)
    	{
    		//getLowMemory().updateSingleMemory(now);
    		getHighMemory().updateSingleMemory(now);
    	}
        else
        {
        	//silentUpdateMemory(lowMemory,now);
        	getHighMemory().silentUpdateMemory(now);
        }
        
    }
    
    /**
	 * Computes the distances to its immediate neighbors
	 */
    float[] getDistances(int mem_id)
    {
    	float[] dist = new float[2];
    	float prev_distance, next_distance = 0.0f;
    	if(mem_id == 1)
    	{
    		next_distance = computeNeuronDistance(index, index+1);
    		prev_distance = computeNeuronDistance(index, index-1);
    	}
    	else
    	{
    		next_distance = computeNeuronDistance(index,index-1);
    		prev_distance = computeNeuronDistance(index, index+1);
    	}
    	dist[0] = next_distance;
    	dist[1] = prev_distance;
    	return dist;
    }
	
	public static void setTauMP(int MPTimeConstantUs) {
		
	}

	public static void setRadius(int radius) {
		RADIUS = radius;		
	}

	public int getRadiusNum() {
		return radiusNum;
	}

	public void setRadiusNum(int radiusNum) {
		this.radiusNum = radiusNum;
	}
	
	public int computePolarity()
	{
		return (onEdge < offEdge)?0:1;
	}

	
	/**
	 * Gets the low memory bank
	 */
	public MemoryBank getLowMemory() {
		return lowMemory;
	}

	public void setLowMemory(MemoryBank lowMemory) {
		this.lowMemory = lowMemory;
	}

	
	
	public MemoryBank getHighMemory() {
		return highMemory;
	}

	public void setHighMemory(MemoryBank highMemory) {
		this.highMemory = highMemory;
	}
	
	
	public static void setFilter(
			AbstractImageReceiver filter) {
		if(filter instanceof ImageReceiver4)
			Neuron.filter = (ImageReceiver4) filter;
	}
	
	
	public static AbstractImageReceiver getFilter()
	{
		return filter;
	}

	/**
	 * @return the startAccumulationTime
	 */
	public int getStartAccumulationTime() {
		return startAccumulationTime;
	}

	/**
	 * @param startAccumulationTime the startAccumulationTime to set
	 */
	public void setStartAccumulationTime(int startAccumulationTime) {
		this.startAccumulationTime = startAccumulationTime;
	}

	public void updateMemory() {
    	if(index!= 1)
    	{
    		try
        	{
        		/** Copy signals from low temp memory to high real memory */
            	if(lowMemory.isScheduledForCopy())
            	{
            		highMemory.realMemory.copyFromMemory(lowMemory.tempMemory);
            		if(filter.LEVEL == FilterLevel.DEBUG)
            			System.out.println("UPDATE MEMORY: "+"NEURON:"+cellNumber+".Erasing lowMemory.tempMemory");
            		lowMemory.tempMemory.eraseMemory();
            		lowMemory.scheduleForCopy(false);
            	}
            	/** Copy signals from high temp memory to low real memory */
            	else if(highMemory.isScheduledForCopy())
            	{
            		lowMemory.realMemory.copyFromMemory(highMemory.tempMemory);
            		if(filter.LEVEL == FilterLevel.DEBUG)
            			System.out.println("UPDATE MEMORY: "+"NEURON:"+cellNumber+".Erasing highMemory.tempMemory");
            		highMemory.tempMemory.eraseMemory();
            		highMemory.scheduleForCopy(false);
            	}
        	}
        	catch(CloneNotSupportedException e)
        	{
        		e.printStackTrace();
        	}
    	}
		
	}
	
	public LinkedList<Neuron> getRadius()
	{
		return rNeurons;
	}
		
}