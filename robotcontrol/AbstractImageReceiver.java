package jaer.myjaer.robotcontrol;

import jaer.myjaer.neuron.Memory;
import jaer.myjaer.neuron.Neuron;
import jaer.myjaer.neuron.Signal;

import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Float;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.HashSet;
import java.util.Hashtable;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.Observable;
import java.util.Observer;

import javax.media.opengl.GL;
import javax.media.opengl.GLAutoDrawable;

import com.sun.opengl.util.GLUT;

import net.sf.jaer.chip.AEChip;
import net.sf.jaer.event.EventPacket;
import net.sf.jaer.event.PolarityEvent;
import net.sf.jaer.eventprocessing.EventFilter2D;
import net.sf.jaer.graphics.FrameAnnotater;

abstract public class AbstractImageReceiver extends EventFilter2D implements FrameAnnotater, Observer {

	/**
	 * The Workstation-Side Controller
	 * @see Controller
	 */
	protected Controller controller;
	
	protected boolean justStarted;
	
	/**
	 * The data class. Contains intermediate data that is passed between the filter and the controller
	 * @see Data
	 */
	protected Data data;
	
	/**
	 * @author vanxa
	 * Operation mode for the OF algorithm. 
	 * Current modes are Debug, Optimize, Normal and Evaluation
	 * Normal operation is used during actual system operation
	 * In Debug mode only 2 radii are created and is used to debug the logic flow of the OF algorithm
	 * In Evaluation mode, procedures are run to evaluate the algorithm
	 * In Optimize mode, the filter variables are optimized: PROTOTYPE 
	 */
	public static enum FilterLevel { DEBUG, OPTIMIZE, NORMAL, EVALUATION };

	/**
	 * Current level
	 */
	public static FilterLevel LEVEL = FilterLevel.NORMAL;
	
	/**
	 * Switch for additional evaluation tasks
	 * 0: None
	 * 1: Light
	 */
	public static final int ADDITIONAL_EVALUATION_TASKS = 0; 
	
	/**
	 * Draws the neural network
	 */
	protected boolean drawNeurons = getPrefs().getBoolean("ImageReceiver.drawNeurons", false);
	
	/**
	 * Draws the velocity vectors that are generated
	 */
	protected boolean drawVectors = getPrefs().getBoolean("ImageReceiver.drawVectors", false);
	
	/**
	 * Displays firing neurons in different color
	 */
	protected boolean drawFiringNeurons = getPrefs().getBoolean("ImageReceiver.drawFiringNeurons", false);
	
	/**
	 * Displays the image processing boundaries. 
	 */
	protected boolean drawProcessBoundaries = getPrefs().getBoolean("ImageReceiver.drawProcessBoundaries", false);
	
	/**
	 * Displays the two neural lines located at the far ends of the image
	 */
	protected boolean drawNeuronLines = getPrefs().getBoolean("ImageReceiver.drawNeuronLines", false);
		
	/**
	 * Default angle between two radii
	 */
	protected static int DEFAULT_ANGLE = 8;
	
	/**
	 * Default number of neurons per radius
	 */
	protected static int DEFAULT_NEURONS_RADIUS = 8;//10
	
	/**
	 * Default time constant for neuron's membrane potential
	 */
	protected static final int DEFAULT_MPTIMECONSTANTUS = 30000;
	
	/**
	 * Default threshold for resetting expected signal polarity
	 */
	protected static final int DEFAULT_SIGNALPOLARITYRESETTHRESH = 500000;
	
	/**
	 * Default threshold for resetting stale neurons
	 */
	protected static final int DEFAULT_NEURONSTALERESET = 2000000;
	
	/**
	 * Default membrane potential threshold
	 */
	protected static final int MPTHRESHOLD = 3;//2
	
	/**
	 * Default receptive field size for every neuron (in pixels)
	 */
	protected static final int RECEPTIVEFIELDSIZEPIXELS = 3;
	
	/**
	 * Default jump amount of the membrane potential of a firing neuron
	 */
	protected static final float MPAFTERFIRING = 10.0f;
	
	/**
	 * Default arrival time threshold for an expected signal
	 */
	protected static final int DEFAULT_ARRTIMETHRESH = 120000;//131000;
	
	/**
	 * Default priority threshold for a signal
	 */
	protected static final int DEFAULT_PRITHRESH = 2;
	
	/**
	 * Default confirmation threshold for a signal
	 */
	protected static final int DEFAULT_CONFIRMTHRESH = 2;
	
	/**
	 * Default angle between two radii
	 */
	protected static final float DEFAULT_EXPINITVELOCITY = 30f;//28.4453f; 

	/**
	 * Default velocity error threshold
	 */
	protected static final float DEFAULT_VELOCITYERRORTHRESHOLD = 0.7f;//0.45f;
	
	/**
	 * Default conlusion depth
	 */
	protected static final int DEFAULT_POSITION_CONCLUSION_DEPTH = 4;
		
	/**
	 * Default signal counter threshold, used for controlling the filter-controller semaphore
	 */
	protected static final int DEFAULT_COMPLETEONCONFIRMEDSIGNALS = 10;
	
	/**
	 * Default weights for velocities
	 */
	protected static final float DEFAULT_VELOCITY_LEFT_WEIGHT = 1f;
	protected static final float DEFAULT_VELOCITY_RIGHT_WEIGHT = 1f;
	
	/**
	 * Radius of the retina: it's half the width of the image, 64 pixels
	 */
	public static final int RADIUS = 64;
	
	/**
	 * Weights for the left and right velocity computations. Used to simulate the horizontal movement of texture patterns in
	 * the bee experiment
	 */
	protected float velocityLeftWeight = getPrefs().getFloat("ImageReceiver.velocityLeftWeight", DEFAULT_VELOCITY_LEFT_WEIGHT);
	protected float velocityRightWeight = getPrefs().getFloat("ImageReceiver.velocityLeftWeight", DEFAULT_VELOCITY_RIGHT_WEIGHT);
	
	/**
	 * The velocity error threshold
	 * Defines the control output. 
	 */
	protected float velocityErrorThreshold = getPrefs().getFloat("ImageReceiver.velocityErrorThreshold", DEFAULT_VELOCITYERRORTHRESHOLD);
	
	/**
	 * The signal arrival threshold
	 * Defines the time window around the expected arrival time of a signal
	 */
	protected int arrivalThreshold = getPrefs().getInt("ImageReceiver.arrivalThreshold", DEFAULT_ARRTIMETHRESH);
	
	/**
	 * The signal priority threshold
	 * Defines how long a late signal will be kept in memory before being removed
	 */
	protected int priorityThreshold = getPrefs().getInt("ImageReceiver.priorityThreshold", DEFAULT_PRITHRESH);
	
	/**
	 * The signal confirmation threshold
	 * Determines how many neurons must receive a signal before that signal is confirmed
	 */
	protected int confirmThreshold = getPrefs().getInt("ImageReceiver.confirmThreshold", DEFAULT_CONFIRMTHRESH);
	
	/**
	 * The expected initial velocity of a signal. It's not known, since there is no prior information, so a random value is set
	 * Can be fine-tuned
	 */
	protected float expectedInitialSignalVelocity = getPrefs().getFloat("ImageReceiver.expectedInitialSignalVelocity",	DEFAULT_EXPINITVELOCITY);
	
	/**
	 * The length of the minor axis of the ellipse, defining the image processing boundaries. 
	 */
	protected float semiMinorAxis = getPrefs().getFloat("ImageReceiver.semiMinorAxis", 64f);
	
	/**
	 * Angle between two radii, in degrees
	 */
	protected int angle = getPrefs().getInt("ImageReceiver.angle",DEFAULT_ANGLE);
	
	/**
	 * The number of neurons per radius
	 * Each time this value is changed, the network is re-constructed to apply the changes
	 */
	 protected int neuronsOnRadius = getPrefs().getInt("ImageReceiver.neuronsOnRadius",DEFAULT_NEURONS_RADIUS);
	
	/** Time constant of LIF neurons membrane potential. It decays exponetially unless a new event is added. */
    protected int MPTimeConstantUs = getPrefs().getInt("ImageReceiver.MPTimeConstantUs", DEFAULT_MPTIMECONSTANTUS);
    
    /**
     * Threshold to reset a neuron memory's signal polarity switch flag .
     * The signal polarity switch flag determines if a signal is to be transmitted to the neighboring neuron or not,
     * depending on whether a same-polarity signal has been transmitted within the time period controller by this threshold.
     */
    protected int signalPolarityResetThreshold = getPrefs().getInt("ImageReceiver.signalPolarityResetThreshold", DEFAULT_SIGNALPOLARITYRESETTHRESH);
    
    /** Life time of LIF neuron */
    protected int neuronStaleReset = getPrefs().getInt("ImageReceiver.neuronStaleReset", DEFAULT_NEURONSTALERESET);

    /**
     * threshold of membrane potential required for firing.
     */
    protected int MPThreshold = getPrefs().getInt("ImageReceiver.MPThreshold", MPTHRESHOLD);
    
    /**
     * size of the receptive field of an LIF neuron.
     */
    protected int receptiveFieldSizePixels = getPrefs().getInt("ImageReceiver.receptiveFieldSizePixels", RECEPTIVEFIELDSIZEPIXELS);
    
    /**
     * conclusion depth for summing up intermediate errors before producing a final conclusion
     */
    protected int conclusionDepth = getPrefs().getInt("ImageReceiver.conclusionDepth", DEFAULT_POSITION_CONCLUSION_DEPTH);
    
    /**
     * Controls the communication between the controller and the filter. The controller waits until at least 
     * this amount of signals have been confirmed
     */
    protected int completeOnConfirmedSignals = getPrefs().getInt("ImageReceiver.completeOnConfirmedSignals", DEFAULT_COMPLETEONCONFIRMEDSIGNALS);
    
    /**
     * switch for controlling the signal traffic. A neuron responds to an edge by sending a signal only once
     */
    protected int polairtySwitch = getPrefs().getInt("ImageReceiver.polaritySwitch", 4);
    
    /**
     * Membrane potential of a neuron jumps down by this amount after firing.
     */
    protected float MPJumpAfterFiring = getPrefs().getFloat("ImageReceiver.MPJumpAfterFiring", MPAFTERFIRING);	
    	
    /**
	 * Stores the neurons whose memory needs to be updated
	 */  
 	protected LinkedList<Neuron> memoryUpdates = new LinkedList<Neuron>(); // Must be performed after check_memory has finished
 	
 	/**
	 * The radii
	 */
 	protected ArrayList<LinkedList<Neuron>> radii = new ArrayList<LinkedList<Neuron>>();
 	
 	/**
	 * The number of radii is defined by the angle and is then used to populate the radii list
	 */
 	protected int numberRadii;
 	
 	/** last update time. It is the timestamp of the latest event. */
    protected int lastTime; 
	
    /**
	 * All neurons in the network, excluding the the ones on the two lines
	 */
	protected ArrayList<Neuron> neurons = new ArrayList<Neuron>();
	
	/**
	 * All neurons from the left line
	 */
	protected ArrayList<Neuron> neuronLineLeft = new ArrayList<Neuron>();
	
	/**
	 * All neurons from the right line
	 */
	protected ArrayList<Neuron> neuronLineRight = new ArrayList<Neuron>();
    
    /** Contains all firing neurons at a specific iteration. Used for display */
    protected HashSet<Integer> firingNeurons = new HashSet<Integer>();
    
    /** Used for display */
    protected Hashtable<Point2D.Float,Point2D.Float> opticalFlowVectors = new Hashtable<Point2D.Float,Point2D.Float>();
    
    /** DVS Chip instance  */
    protected AEChip mychip;

    /** Optimization */
 	protected int optimCase;
 	protected float optimizationValue;
 	protected float optimValueChange;
 	protected Hashtable<java.lang.Float,java.lang.Float> optimMap;
 	protected float optimumScore;
 	protected float optimFinish;
 	protected OptimizationCombination bestCombination;
 	 	
 	protected int iterationCounter;
 	protected boolean isJustStarted = true;
 	protected boolean isSwitchingCase = true;
 	
 	/**
	 * Evaluation variables
	 */
	protected int evaluationCase;
	protected float evaluationValue;
	protected float evaluationValueChange;
	protected float evaluationValueFinish;
	protected float evaluationErrorRatio;
	protected int numberEvents;
    protected int numberConfirmedSignals;
    protected int numberCreatedSignals;	
    protected int numberFiringNeurons;
    protected float ratioLeftRightSignals;
    
    /**
     * Evaluation file writer
     */
    protected String evaluationFilename= "/home/vanxa/workspace/Project/evaluation/evaluation_results.txt";
    protected FileWriter evaluationWriter;
	protected BufferedWriter evaluationBuffWriter;
	
	/**
	 * Optimization file writer
	 */
	protected String optimizationFilename= "/home/vanxa/workspace/Project/evaluation/optimization.txt";
    protected FileWriter optimWriter;
	protected BufferedWriter optimBuffWriter;
    
	/**
	 * Constructor
	 * @param chip
	 */
	public AbstractImageReceiver(AEChip chip) {
		super(chip);
		this.mychip = chip;
		Neuron.setRadius(RADIUS);
		if(LEVEL == FilterLevel.EVALUATION)
        {
        	System.out.println("EVALUATION: ON");
        	try 
            {
        		evaluationWriter = new FileWriter(evaluationFilename);
        		optimWriter = new FileWriter(optimizationFilename);
    		} 
            catch (IOException e) 
            {
    			e.printStackTrace();
    		}
        	evaluationBuffWriter = new BufferedWriter(evaluationWriter);
        	optimBuffWriter = new BufferedWriter(optimWriter);
            DEFAULT_NEURONS_RADIUS = 10;            
        }		
		else if(LEVEL == FilterLevel.DEBUG)
        {
        	DEFAULT_ANGLE = 168;
        	DEFAULT_NEURONS_RADIUS = 8;
        }
		else if(LEVEL == FilterLevel.OPTIMIZE)
		{
			DEFAULT_NEURONS_RADIUS = 10;
		}
        else
        {
        	DEFAULT_ANGLE = 8;
        	DEFAULT_NEURONS_RADIUS = 10;
        }
        
        /**
         * Don't call setAngle or setNeuronsOnRadius here, because these methods calls initFilter(), something I don't want at this stage of initialization
         */
        this.angle = DEFAULT_ANGLE; 
        this.neuronsOnRadius = DEFAULT_NEURONS_RADIUS;
        
        Runtime.getRuntime().addShutdownHook(new Thread(){
            @Override
            public void run (){
               log.info("Closing files");
               try 
               {
            	   evaluationBuffWriter.close();
            	   optimBuffWriter.close();
               } 
               catch (IOException e) 
               {
            	   e.printStackTrace();
               }
               log.info("Shutting down controller...");
               controller.shutdown();
               try {
            	   Thread.sleep(2000);
               } catch (InterruptedException e) {
            	   e.printStackTrace();
               }
            }
        }); 
        
	}
	
	/**
	 * Reset
	 */
	public void resetValues()
	{
		setMPTimeConstantUs(DEFAULT_MPTIMECONSTANTUS);
        setSignalPolarityResetThreshold(DEFAULT_SIGNALPOLARITYRESETTHRESH);
        setMPThreshold(MPTHRESHOLD);
        setReceptiveFieldSizePixels(RECEPTIVEFIELDSIZEPIXELS);
        setMPJumpAfterFiring(MPJumpAfterFiring);
        setAngle(DEFAULT_ANGLE);
        setNeuronsOnRadius(DEFAULT_NEURONS_RADIUS);
        setNeuronStaleReset(DEFAULT_NEURONSTALERESET);
        setArrivalThreshold(DEFAULT_ARRTIMETHRESH);
    	setPriorityThreshold(DEFAULT_PRITHRESH);
    	setConfirmThreshold(DEFAULT_CONFIRMTHRESH);
    	setExpectedInitialSignalVelocity(DEFAULT_EXPINITVELOCITY);
    	setVelocityErrorThreshold(DEFAULT_VELOCITYERRORTHRESHOLD);
    	setCompleteOnConfirmedSignals(DEFAULT_COMPLETEONCONFIRMEDSIGNALS);
    	setConclusionDepth(DEFAULT_POSITION_CONCLUSION_DEPTH);    	
	}
	
	/**
	 * 
	 * @param drawNeurons
	 */
	synchronized public void setDrawNeurons(boolean drawNeurons)
    {
    	this.drawNeurons = drawNeurons;
    	getPrefs().putBoolean("ImageReceiver.drawNeurons", drawNeurons);
    }
    
	/**
	 * 
	 * @return
	 */
    synchronized public boolean getDrawNeurons()
    {
    	return drawNeurons;
    }
     
    /**
     * 
     * @param drawVectors
     */
    synchronized public void setDrawVectors(boolean drawVectors)
    {
    	this.drawVectors = drawVectors;
    	getPrefs().putBoolean("ImageReceiver.drawVectors", drawVectors);
    }
    
    /**
     * 
     * @return
     */
    synchronized public boolean getDrawVectors()
    {
    	return drawVectors;
    }
    
    /**
     * 
     * @param drawFiringNeurons
     */
    synchronized public void setDrawFiringNeurons(boolean drawFiringNeurons)
    {
    	this.drawFiringNeurons = drawFiringNeurons;
    	getPrefs().putBoolean("ImageReceiver.drawFiringNeurons", drawFiringNeurons);
    }
    
    /**
     * 
     * @return
     */
    synchronized public boolean getDrawFiringNeurons()
    {
    	return drawFiringNeurons;
    }
    
    /**
     * 
     * @return
     */
    synchronized public int getAngle() {
        return angle;
    }

    /**
     * set receptiveFieldSizePixels
     *
     * @param receptiveFieldSizePixels
     */
    synchronized public void setAngle(int angle) {
    	if(angle > 0)
    	{
    		this.angle = angle;
            getPrefs().putInt("ImageReceiver.angle", angle);
            initFilter();
    	}
    	
    }
    
    /**
     * 
     * @return
     */
    synchronized public int getNeuronsOnRadius() {
        return neuronsOnRadius;
    }

    /**
     * set receptiveFieldSizePixels
     *
     * @param receptiveFieldSizePixels
     */
    synchronized public void setNeuronsOnRadius(int neuronsOnRadius) {
        if(neuronsOnRadius > 2)
        {
        	this.neuronsOnRadius = neuronsOnRadius;
            getPrefs().putInt("ImageReceiver.neuronsOnRadius", neuronsOnRadius);
        }
    	                
    }
    
	/*
	 *     
	 */
	public int getNeuronStaleReset() {
        return neuronStaleReset;
    }

    /**
     * sets neuronLifeTimeUs
     *
     * @param neuronLifeTimeUs
     */
    public void setNeuronStaleReset(int neuronStaleReset) {
        this.neuronStaleReset = neuronStaleReset;
        getPrefs().putInt("ImageReceiver.neuronStaleReset", neuronStaleReset);
    }
 
    /**
     * 
     * @param arrivalThreshold
     */
    public void setArrivalThreshold(int arrivalThreshold)
    {
    	this.arrivalThreshold = arrivalThreshold;
        getPrefs().putInt("ImageReceiver.arrivalThreshold", arrivalThreshold);
    }
    
    /**
     * 
     * @return
     */
    public int getArrivalThreshold()
    {
    	return arrivalThreshold;
    }
    
    /**
     * 
     * @param priorityThreshold
     */
    public void setPriorityThreshold(int priorityThreshold)
    {
    	this.priorityThreshold = priorityThreshold;
        getPrefs().putInt("ImageReceiver.priorityThreshold", priorityThreshold);
    }
    
    /**
     * 
     * @return
     */
    public int getPriorityThreshold()
    {
    	return priorityThreshold;
    }
    
    /**
     * 
     * @param confirmThreshold
     */
    public void setConfirmThreshold(int confirmThreshold)
    {
    	this.confirmThreshold = confirmThreshold;
        getPrefs().putInt("ImageReceiver.confirmThreshold", confirmThreshold);
    }
    
    /**
     * 
     * @return
     */
    public int getConfirmThreshold()
    {
    	return confirmThreshold;
    }
    
    /**
     * 
     * @param expectedInitialSignalVelocity
     */
    public void setExpectedInitialSignalVelocity(float expectedInitialSignalVelocity)
    {
    	this.expectedInitialSignalVelocity = expectedInitialSignalVelocity;
        getPrefs().putFloat("ImageReceiver.expectedInitialSignalVelocity", expectedInitialSignalVelocity);
    }
    
    /**
     * 
     * @return
     */
    public float getExpectedInitialSignalVelocity()
    {
    	return expectedInitialSignalVelocity;
    }
    
    /**
     * 
     * @param semiMinorAxis
     */
    public void setSemiMinorAxis(float semiMinorAxis)
    {
    	if(semiMinorAxis <= 64)
    	{
    		this.semiMinorAxis = semiMinorAxis;
    		getPrefs().putFloat("ImageReceiver.semiMinorAxis", semiMinorAxis);
    	}
    }
    
    /**
     * 
     * @return
     */
    public float getSemiMinorAxis()
    {
    	return semiMinorAxis;
    }
    
    /**
     * returns the time constant of the neuron's membranePotential
     *
     * @return
     */
    public int getMPTimeConstantUs() {
        return MPTimeConstantUs;
    }

    /**
     * sets MPTimeConstantUs
     *
     * @param
     */
    public void setMPTimeConstantUs(int MPTimeConstantUs) {
        this.MPTimeConstantUs = MPTimeConstantUs;
        getPrefs().putInt("ImageReceiver.MPTimeConstantUs", MPTimeConstantUs);
        Neuron.setTauMP(MPTimeConstantUs);
    }

    /**
     * returns neuronLifeTimeUs
     *
     * @return
     */
    public int getSignalPolarityResetThreshold() {
        return signalPolarityResetThreshold;
    }

    /**
     * sets neuronLifeTimeUs
     *
     * @param neuronLifeTimeUs
     */
    public void setSignalPolarityResetThreshold(int signalPolarityResetThreshold) {
        this.signalPolarityResetThreshold = signalPolarityResetThreshold;
        getPrefs().putInt("ImageReceiver.signalPolarityResetThreshold", signalPolarityResetThreshold);
    }

    public void setCompleteOnConfirmedSignals(int completeOnConfirmedSignals)
    {
    	if(completeOnConfirmedSignals > 0)
    	{
    		this.completeOnConfirmedSignals =	completeOnConfirmedSignals;
        	getPrefs().putInt("ImageReceiver.completeOnConfirmedSignals", completeOnConfirmedSignals);
        }
    }
    
    public int getCompleteOnConfirmedSignals()
    {
    	return completeOnConfirmedSignals;
    }
    
    public void setConclusionDepth(int conclusionDepth)
    {
    	this.conclusionDepth = conclusionDepth;
    	getPrefs().putInt("ImageReceiver.conclusionDepth", conclusionDepth);
    }
    
    public int getConclusionDepth()
    {
    	return conclusionDepth;
    }
    
    
    
    /**
     * returns receptiveFieldSizePixels
     *
     * @return
     */
    public int getReceptiveFieldSizePixels() {
        return receptiveFieldSizePixels;
    }

    /**
     * set receptiveFieldSizePixels
     *
     * @param receptiveFieldSizePixels
     */
    synchronized public void setReceptiveFieldSizePixels(int receptiveFieldSizePixels) {
		if(receptiveFieldSizePixels < 1)
            this.receptiveFieldSizePixels = 1;
		else
		{
			this.receptiveFieldSizePixels = receptiveFieldSizePixels;
	        getPrefs().putInt("ImageReceiver.receptiveFieldSizePixels", receptiveFieldSizePixels);
	    }
		initFilter();
        
    }

    /**
     * returns MPThreshold
     *
     * @return
     */
    public int getMPThreshold() {
        return MPThreshold;
    }

    /**
     * sets MPThreshold
     *
     * @param MPThreshold
     */
    public void setMPThreshold(int MPThreshold) {
        this.MPThreshold = MPThreshold;
        getPrefs().putInt("ImageReceiver.MPThreshold", MPThreshold);
    }
    
    /**
     * returns the last timestamp ever recorded at this filter
     *
     * @return the last timestamp ever recorded at this filter
     */
    public int getLastTime() {
        return lastTime;
    }

    /**
     * returns MPJumpAfterFiring
     *
     * @return
     */
    public float getMPJumpAfterFiring() {
        return MPJumpAfterFiring;
    }

    /**
     * sets MPJumpAfterFiring
     *
     * @param MPJumpAfterFiring
     */
    public void setMPJumpAfterFiring(float MPJumpAfterFiring) {
        this.MPJumpAfterFiring = MPJumpAfterFiring;
        getPrefs().putFloat("ImageReceiver.MPJumpAfterFiring", MPJumpAfterFiring);
    }
    
    /**
     * 
     * @param velocityErrorThreshold
     */
    public void setVelocityErrorThreshold(float velocityErrorThreshold)
    {
    	this.velocityErrorThreshold = velocityErrorThreshold;
    	getPrefs().putFloat("ImageReceiver.velocityErrorThreshold", velocityErrorThreshold);
    }
    
    /**
     * 
     * @return
     */
    public float getVelocityErrorThreshold()
    {
    	return velocityErrorThreshold;
    }
    
    /**
     * 
     * @return
     */
    public Controller getControllerInstance()
    {
    	return controller;
    }
    
    /**
     * 
     * @return
     */
    public Data getDataInstance()
    {
    	return data;
    }
    
    /**
     * 
     * @return
     */
    public Hashtable<Point2D.Float,Point2D.Float> getOpticalFlowVectors()
    {
    	return opticalFlowVectors;
    }
    
    /**
     * 
     * @return
     */
    public LinkedList<Neuron> getMemoryUpdateList()
    {
    	return memoryUpdates;
    }
    
    /**
     * 
     * @param numNeuron
     * @param cos
     * @param sin
     * @return
     */
    public Point2D.Float computeNeuronLocation(int numNeuron, double cos, double sin)
	{
		float pos = (float)(RADIUS*numNeuron*(numNeuron+1))/(float)((neuronsOnRadius-1)*neuronsOnRadius);
		Point2D.Float location = new Point2D.Float(64+(float)cos*pos,64+(float)sin*pos);
		return location;
		
	}
    
    /**
     * 
     */
    @Override
	public void annotate(GLAutoDrawable drawable) {
		
		if (!isFilterEnabled()) {
            return;
        }	
		
		GL gl=drawable.getGL();
		
		if(drawVectors)
		{
			Enumeration<Point2D.Float> vector_iterator = opticalFlowVectors.keys();
			gl.glColor3f(1.0f,0.0f,0.0f);
			while(vector_iterator.hasMoreElements())
			{
				Point2D.Float source = vector_iterator.nextElement();
				Point2D.Float vec = opticalFlowVectors.get(source);
				
				
				gl.glBegin(GL.GL_QUADS);
				gl.glVertex2f(source.x-1,source.y-1);
				gl.glVertex2f(source.x-1,source.y+1);
				gl.glVertex2f(source.x+1,source.y+1);
				gl.glVertex2f(source.x+1,source.y-1);
				gl.glEnd();
				
				gl.glBegin(GL.GL_LINES);
				gl.glVertex2f(source.x,source.y);
				gl.glVertex2f(vec.x,vec.y);
				gl.glEnd();
			}
			
		}
		
		if(drawNeurons)
		{
			gl.glColor3f(0.0f,1.0f,0.0f);
			float spreadOut = (neurons.isEmpty()) ? 
								0: 
								120f/neurons.size();
			float yCoordinate = 120f;
			float xCoordinate = (angle < 90) ? 
								3:
								67;
			for(Neuron neuron:neurons)
			{
				
				Float loc = neuron.getLocation();
				gl.glBegin(GL.GL_QUADS);
				gl.glVertex2f(loc.x-1, loc.y);
				gl.glVertex2f(loc.x+1,loc.y);
				gl.glVertex2f(loc.x-1,loc.y+1);
				gl.glVertex2f(loc.x+1,loc.y+1);
				gl.glEnd();
				if(LEVEL == FilterLevel.DEBUG)
				{
					gl.glRasterPos3f(loc.x-3,loc.y+3,0);
					chip.getCanvas().getGlut().glutBitmapString(GLUT.BITMAP_HELVETICA_12,String.format("%d",neuron.getCellNumber()));
					if(neuron.getIndex() == neuron.getRadius().size())
					{
						gl.glRasterPos3f(loc.x,loc.y+10,0);
						chip.getCanvas().getGlut().glutBitmapString(GLUT.BITMAP_HELVETICA_12,String.format("%d",neuron.getRadiusNum()));
					}
					
					gl.glRasterPos2f(xCoordinate,yCoordinate);
					/**
					 * Display stats on opposite side of the image: assuming only one radius is present
					 */
					String neuronStats = ""+neuron.getCellNumber()+":";
					
					try
					{
						Memory temp = neuron.getLowMemory().getTempMemory();
						neuronStats += "LT:";
						for(int label : temp.getSignalLabels())
						{
							Signal signal = temp.getSignals().get(label); 
							if(signal.getCounter() > confirmThreshold)
								neuronStats += "(!)";
							String labelString = Integer.toString(label);
							neuronStats += labelString.substring(labelString.length()-3)+"("+signal.getPolarity()+":c"+signal.getCounter()+");";
						}
						
					}
					catch(NullPointerException exc) {}
					try
					{
						Memory real = neuron.getHighMemory().getRealMemory();
						neuronStats += "HR:";
						for(int label: real.getSignalLabels())
						{
							Signal signal = real.getSignals().get(label); 
							if(signal.getCounter() > confirmThreshold)
								neuronStats += "(!)";
							String labelString = Integer.toString(label);
							neuronStats += labelString.substring(labelString.length()-3)+"("+signal.getPolarity()+":c"+signal.getCounter()+");";
						}
					}
					catch(NullPointerException exc) {}
					
					chip.getCanvas().getGlut().glutBitmapString(GLUT.BITMAP_HELVETICA_12,neuronStats);
					yCoordinate -= spreadOut;
					
					
					gl.glRasterPos3f(loc.x-10,loc.y-10,0);
					chip.getCanvas().getGlut().glutBitmapString(GLUT.BITMAP_HELVETICA_12,"RFS:"+neuron.getReceptiveFieldSize());
					
					Point2D.Float edgePosition = neuron.computeAndReturnEdgePosition();
					if(edgePosition!= null)
					{
						gl.glColor3f(1.0f,0.0f,0.0f);
						gl.glBegin(GL.GL_QUADS);
						gl.glVertex2f(edgePosition.x-1, edgePosition.y-1);
						gl.glVertex2f(edgePosition.x-1, edgePosition.y+1);
						gl.glVertex2f(edgePosition.x+1, edgePosition.y+1);
						gl.glVertex2f(edgePosition.x+1, edgePosition.y-1);
						gl.glEnd();
						gl.glRasterPos3f(edgePosition.x+2,loc.y+2,0);
						chip.getCanvas().getGlut().glutBitmapString(GLUT.BITMAP_HELVETICA_12,Integer.toString(neuron.getCellNumber()));
						gl.glColor3f(0.0f,0.0f,1.0f);
					}
					
				}
				
			}
		}
		if(drawFiringNeurons)
		{
			
			Iterator<Integer> itr = firingNeurons.iterator();
			while(itr.hasNext())
			{
				Neuron firing_neuron = neurons.get(itr.next());
				Float loc = firing_neuron.getLocation();
				gl.glColor3f(0.0f,1.0f,0.0f);
				gl.glBegin(GL.GL_QUADS);
				gl.glVertex2f(loc.x-1, loc.y);
				gl.glVertex2f(loc.x+1,loc.y);
				gl.glVertex2f(loc.x-1,loc.y+1);
				gl.glVertex2f(loc.x+1,loc.y+1);
				gl.glEnd();
			}
		}
		
		if(drawProcessBoundaries)
		{
			gl.glBegin(GL.GL_LINE_LOOP);
			/*for(float yCoor = 13f;yCoor <= 115f; yCoor++)
			{
				float xCoor = -64f/(51f*51f)*(yCoor-64f)*(yCoor-64f) + 64f ;
				gl.glVertex2f(xCoor, yCoor);
			}
			gl.glEnd();
			gl.glBegin(GL.GL_LINE_LOOP);
			for(float yCoor = 13f;yCoor <= 115f; yCoor++)
			{
				float xCoor = 64f/(51f*51f)*(yCoor-64f)*(yCoor-64f) + 64f ;
				gl.glVertex2f(xCoor, yCoor);
			}
			gl.glEnd();*/
			for(float yCoor = 64 - semiMinorAxis;yCoor <= 64 + semiMinorAxis; yCoor++)
			{
				float xCoor = -64f/(semiMinorAxis*semiMinorAxis)*(yCoor-64f)*(yCoor-64f) + 64f ;
				gl.glVertex2f(xCoor, yCoor);
			}
			gl.glEnd();
			gl.glBegin(GL.GL_LINE_LOOP);
			for(float yCoor = 64 - semiMinorAxis;yCoor <= 64 + semiMinorAxis; yCoor++)
			{
				float xCoor = 64f/(semiMinorAxis*semiMinorAxis)*(yCoor-64f)*(yCoor-64f) + 64f ;
				gl.glVertex2f(xCoor, yCoor);
			}
			gl.glEnd();
		}
		
		if(drawNeuronLines)
		{
			for(int index = 0; index < neuronLineLeft.size(); index++)
			{
				Neuron leftNeuron = neuronLineLeft.get(index);
				Float locLeft = leftNeuron.getLocation();
				gl.glColor3f(1.0f,0.0f,0.0f);
				gl.glBegin(GL.GL_QUADS);
				gl.glVertex2f(locLeft.x-1, locLeft.y);
				gl.glVertex2f(locLeft.x+1,locLeft.y);
				gl.glVertex2f(locLeft.x-1,locLeft.y+1);
				gl.glVertex2f(locLeft.x+1,locLeft.y+1);
				gl.glEnd();
				
				Neuron rightNeuron = neuronLineRight.get(index);
				Float locRight = rightNeuron.getLocation();
				gl.glColor3f(1.0f,0.0f,0.0f);
				gl.glBegin(GL.GL_QUADS);
				gl.glVertex2f(locRight.x-1, locRight.y);
				gl.glVertex2f(locRight.x+1,locRight.y);
				gl.glVertex2f(locRight.x-1,locRight.y+1);
				gl.glVertex2f(locRight.x+1,locRight.y+1);
				gl.glEnd();
			}
		}
				
		
	}
    
    /**
	 * @return the drawProcessBoundaries
	 */
	public boolean getDrawProcessBoundaries() {
		return drawProcessBoundaries;
	}

	/**
	 * @param drawProcessBoundaries the drawProcessBoundaries to set
	 */
	public void setDrawProcessBoundaries(boolean drawProcessBoundaries) {
		this.drawProcessBoundaries = drawProcessBoundaries;
		getPrefs().putBoolean("ImageReceiver.drawProcessBoundaries", drawProcessBoundaries);
	}
	
	/**
	 * @return the drawProcessBoundaries
	 */
	public boolean getDrawNeuronLines() {
		return drawNeuronLines;
	}

	/**
	 * @param drawProcessBoundaries the drawProcessBoundaries to set
	 */
	public void setDrawNeuronLines(boolean drawNeuronLines) {
		this.drawNeuronLines = drawNeuronLines;
		getPrefs().putBoolean("ImageReceiver.drawNeuronLines", drawNeuronLines);
	}
	
	/**
	 * 
	 */
	@Override
	public void update(Observable o, Object arg) {
		if (o == this) {
            UpdateMessage msg = (UpdateMessage) arg;
            updateNeurons(msg.timestamp); // at least once per packet update list
        } 
	}
		
	/**
	 * 
	 * @param time
	 */
	abstract public void updateNeurons(int time);
	
	/**
	 * Will increase the numberConfirmedSignals counter. Used in Evaluation
	 */
	public void increaseConfirmedSignalCounter()
	{
		this.numberConfirmedSignals++;
	}
	
	/**
	 * Will increase the numberCreatedSignals counter, used in Evaluation
	 */
	public void increaseCreatedSignalsCounter()
	{
		this.numberCreatedSignals++;
	}

	public void setVelocityLeftWeight(float velocityLeftWeight)
	{
		getPrefs().putFloat("ImageReceiver.velocityLeftWeight", velocityLeftWeight);
		this.velocityLeftWeight = velocityLeftWeight;
	}
	
	public float getVelocityLeftWeight()
	{
		return velocityLeftWeight;
	}
	
	public void setVelocityRightWeight(float velocityRightWeight)
	{
		getPrefs().putFloat("ImageReceiver.velocityRightWeight", velocityRightWeight);
		this.velocityRightWeight = velocityRightWeight;
	}
	
	public float getVelocityRightWeight()
	{
		return velocityRightWeight;
	}
}

