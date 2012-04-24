/**
 * 
 */
package jaer.myjaer.robotcontrol;

import java.awt.geom.Point2D;
import java.util.LinkedList;
import java.util.Observable;
import java.util.Observer;

import jaer.myjaer.neuron.Neuron;
import jaer.myjaer.neuron.NeuronRing;
import jaer.myjaer.robotcontrol.AbstractImageReceiver.FilterLevel;
import net.sf.jaer.chip.AEChip;
import net.sf.jaer.event.EventPacket;

/**
 * @author vanxa
 *
 */
public class CollisionDetector extends AbstractImageReceiver {

	private LinkedList<NeuronRing> neuronRings;
	private boolean isSecondStage = false;
	private long timeFromFirstStage = 0;
	private int firstIndex = 0;
	private int secondIndex = 0;
		
	protected int ringFireThreshold = getPrefs().getInt("CollisionDetector.ringFireThreshold",4);
	protected int pauseBetweenStages = getPrefs().getInt("CollisionDetector.pauseBetweenStages", 80000);
	
	
	public CollisionDetector(AEChip chip) {
		super(chip);
		neuronRings = new LinkedList<NeuronRing>();
		Neuron.setFilter(this);
		chip.addObserver(this);
        addObserver(this);
        //DEFAULT_ANGLE = 3;
        //DEFAULT_NEURONS_RADIUS = 30;
        this.angle = DEFAULT_ANGLE;
        this.neuronsOnRadius = DEFAULT_NEURONS_RADIUS;
        
	}

	/* (non-Javadoc)
	 * @see net.sf.jaer.eventprocessing.EventFilter#resetFilter()
	 */
	@Override
	public void resetFilter() {
		timeFromFirstStage = 0;
		isSecondStage = false;
		firstIndex = 0;
		secondIndex = 0;	
		initFilter();
	}
	
	synchronized public void setRingFireThreshold(int ringFireThreshold)
	{
		getPrefs().putInt("CollisionDetector.ringFireThreshold", ringFireThreshold);
		this.ringFireThreshold = ringFireThreshold;
	}
	
	synchronized public int getRingFireThreshold()
	{
		return ringFireThreshold;
	}
	
	synchronized public void setPauseBetweenStages(int pauseBetweenStages)
	{
		getPrefs().putInt("CollisionDetector.pauseBetweenStages", pauseBetweenStages);
		this.pauseBetweenStages = pauseBetweenStages;
	}
	
	synchronized public int getPauseBetweenStages()
	{
		return pauseBetweenStages;
	}

	/* (non-Javadoc)
	 * @see net.sf.jaer.eventprocessing.EventFilter#initFilter()
	 */
	@Override
	public void initFilter() {
		neuronRings.clear();
		radii.clear();
		neurons.clear();
		numberRadii = (int)(360/this.angle);
		//setRingFireThreshold((int)numberRadii*3/4);
		for(int radius = 0;radius<numberRadii;radius++)
	    {
			LinkedList<Neuron> rNeurons = new LinkedList<Neuron>();
			radii.add(radius,rNeurons);
	    	double cos,sin = 0f;
	    	double angle = Math.toRadians(radius*this.angle);
	    	cos = Math.cos(angle);
	    	sin = Math.sin(angle);
	    	for(int numNeuron=1;numNeuron <= neuronsOnRadius;numNeuron++)
	    	{
	    		Point2D.Float neuronLocation = computeNeuronLocation(numNeuron,cos,sin);
	    		if(neuronLocation.x <= 128 && neuronLocation.x >= 0 && neuronLocation.y >= 0 && neuronLocation.y <= 128)
	    		{
	    			int cellNumber = radius*(neuronsOnRadius-1) + numNeuron-1;
	    			Neuron neuron = new Neuron(cellNumber, rNeurons, numNeuron, neuronLocation, radius);
	    			if(numNeuron != 1)
	    			{
	    				neuron.setLowNeuron(neurons.get(cellNumber-1));
	    				neurons.get(cellNumber-1).setHighNeuron(neuron);
	    			}
	    			rNeurons.add(numNeuron-1,neuron);
	    			neurons.add(cellNumber,neuron);
	    		}
	        	
	    	}
	    }
		for(int neuronIndex=0;neuronIndex < neuronsOnRadius; neuronIndex++)
		{
			NeuronRing ring = new NeuronRing(neuronIndex);
			neuronRings.add(ring);
			if(neuronIndex != 0)
			{
				ring.setLowRing(neuronRings.get(neuronIndex-1));
				neuronRings.get(neuronIndex-1).setHighRing(ring);
			}
			for(int radiusIndex = 0;radiusIndex < numberRadii; radiusIndex++)
			{
				if(neuronIndex < radii.get(radiusIndex).size())
					ring.addNeuron(radii.get(radiusIndex).get(neuronIndex));
				
			}
		}
	}

	@Override
	synchronized public EventPacket<?> filterPacket(EventPacket<?> in) {
		super.filterPacket(in);
		return in;
	}
	
	public int findHighestFiringRingIndex()
	{
		int maxFiringIndex = -1;
        for(NeuronRing ring : neuronRings)
        {
        	boolean fired = ring.isFired(getRingFireThreshold());
        	/* Will first check lower ring, then upper ring, in order to deduce if two rings have responded on a single moving stimulus */
        	if(fired)
        	{
        		if(maxFiringIndex < ring.getIndex())
        		{
        			maxFiringIndex = ring.getIndex();
        		}
        	}
        }
        return maxFiringIndex;
	}

	@Override
	public void updateNeurons(int t) {
		firingNeurons.clear();
        
    	if (!neurons.isEmpty()) {
            int timeSinceSupport, timeSinceAccumulationStarted;
            for(Neuron neuron: neurons)
            {
            	try {
                    // reset stale neurons
            		timeSinceAccumulationStarted = t- neuron.getStartAccumulationTime();
                    timeSinceSupport = t - neuron.getLastEventTimestamp();
                    if (timeSinceSupport > neuronStaleReset)
                    {
                    	neuron.reset(false);
                    	if(LEVEL == FilterLevel.DEBUG && neuron.isFired())
                    	{
                    		System.out.println("neuron.reset():Radius:"+neuron.getRadiusNum()+";Neuron: "+ neuron.getIndex()+";Time: " + t+"; lastEventTimestamp:" + neuron.getLastEventTimestamp());
                    	}
                    }
                    if(neuron.isAboveThreshold(lastTime))
                    {
                    	firingNeurons.add(neuron.getCellNumber());
                    }
                    
                } catch (java.util.ConcurrentModificationException e) {
                    // this is in case neuron list is modified by real time filter during updating neurons
                    initFilter();
                    log.warning(e.getMessage());
                } 
            }
            
            if(!isSecondStage)
            {
            	firstIndex = findHighestFiringRingIndex();
            	isSecondStage = true;
            	timeFromFirstStage = t;
            	
            }
            else if (isSecondStage && t - timeFromFirstStage >= pauseBetweenStages)
            {
            	secondIndex = findHighestFiringRingIndex();
            	int indexDifference = secondIndex - firstIndex;
            	if(indexDifference > 0)
            	{
            		System.out.println("Object is approaching!");
            		if(getNeuronsOnRadius() > 3 && secondIndex >= getNeuronsOnRadius() - 3)
            		{
            			System.out.println("Collision soon!s");
            		}
            	}
            	else if(indexDifference < 0)
            	{
            		System.out.println("Object is moving away from robot");
            	}
            	else
            	{
            		//System.out.println("Indices are the same");
            	}
            }                                    	
        }
		
	}

}
