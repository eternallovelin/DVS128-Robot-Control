/**
 * @author Ivan Konstantinov
 * 
 */
package jaer.myjaer.robotcontrol;

import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Float;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Hashtable;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.Observable;
import java.util.Observer;
import javax.media.opengl.GL;
import javax.media.opengl.GLAutoDrawable;
import javax.security.auth.callback.ConfirmationCallback;

import com.sun.opengl.util.GLUT;

import net.sf.jaer.chip.AEChip;
import net.sf.jaer.event.EventPacket;
import net.sf.jaer.event.PolarityEvent;
import net.sf.jaer.graphics.FrameAnnotater;

import jaer.myjaer.neuron.*;

public class ImageReceiver4 extends AbstractImageReceiver implements Observer {

	/**
	 * Used for evaluation and optimization. Defines the start position of the robot
	 * 0: right
	 * 1: left
	 * 2: center
	 */
	protected static final int ROBOT_START_POSITION = 0;
	
	/** If I only want to evaluate a single case */
	protected static final boolean EVAL_SINGLE_CASE = true;
	
	/** Used for completeOnConfirmedSignals */
	protected int signalCounter;

	/** The controller uses this information to start the robot (force it to move when idle ) */
	private boolean isRobotIdle = false;
        
    /** DVS Chip instance  */
    private AEChip mychip;
        
	/**
	 * Constructor
	 * @param chip: the AEchip used
	 */
    public ImageReceiver4(AEChip chip) 
	{
		super(chip);
		/** Give access to the main functions of the filter to underlying components */
		Neuron.setFilter(this);
        MemoryBank.setFilter(this);
        Memory.setFilter(this);
        
		chip.addObserver(this);
        addObserver(this);
        
        justStarted = true;
        signalCounter = 0;
                        
        
        /** Initialize controller and data objects */
        //if(LEVEL != FilterLevel.EVALUATION)
        //{
        	data = new Data(getNeuronsOnRadius(), this);
            controller = new Controller(data, this);
            Thread controllerThread = new Thread(controller);
            controllerThread.start();
            controller.setVelocityErrorThreshold(velocityErrorThreshold);
        //}
        initFilter();
     }
	
    /**
     * Main filter
     * If the size of the packet is small, then the robot is idle
     * Otherwise, events are added to the LIF neurons in the network
     * @param in The packet of address-events to be filtered
     */
	synchronized public EventPacket<?> filterPacket(EventPacket<?> in) {
		if(LEVEL == FilterLevel.DEBUG)
		{
			System.out.println("####################################filterPacket()##################################");
		}
		opticalFlowVectors.clear();
		isRobotIdle = in.getSize() < 190 ? true : false; // Should be a small number, corresponding to the number of noisy events
		if(data != null)
		{
			data.setCameraIdle(isRobotIdle); 
		}
		if(isRobotIdle)
			return in;
		
		if(LEVEL == FilterLevel.EVALUATION || LEVEL == FilterLevel.OPTIMIZE)
		{
			iterationCounter++;
			numberEvents += in.getSize();
		}
		
		if((LEVEL == FilterLevel.EVALUATION && !isRobotIdle) || LEVEL != FilterLevel.EVALUATION) // This flag is set to prevent the Evaluation to include frames with few events
		{
			/** Process the packet */
			boolean updatedNeurons = false;
			out = in;
	        if(in == null) 
	        	return out;
	        if(in.getSize() == 0) 
	        	return in;      
	        if(neurons == null) // Make sure neurons are initialized
	        	initFilter();
	        
	        for(Object o:in)
	        {
	        	PolarityEvent pe = (PolarityEvent)o; // Polarity of events is either ON or OFF
	        	lastTime = pe.getTimestamp();
	        	
	        	/** Assign events to the neuron lines: used for correction movement errors */
	        	if(pe.x <10)
	        		assignToNeuronLine(neuronLineLeft,pe);
	        	if(pe.x > 120)
	        		assignToNeuronLine(neuronLineRight, pe);
	        	
	        	/** Find nearest retina radius */
	        	Point2D.Float distance = new Point2D.Float(pe.x - 64,pe.y-64);
	        	double dot_product = (double)(distance.x); // Am using the (1,0) elementary vector
	        	double length = (double)Math.sqrt(distance.x*distance.x+distance.y*distance.y);
	        	double angle; // Using degrees here. This is the angle that the event forms with the X axis
	        	if(distance.y < 0)
	        		angle = 360 - Math.toDegrees(Math.acos(dot_product/length));
	        	else
	        		angle = Math.toDegrees(Math.acos(dot_product/length));
	        	int numRadii = radii.size();
	        	int radius_index = (int)(angle/this.angle) % numRadii; // Find the closest radius to the event
	        	int remainder = (int)(angle % this.angle);
	        	if(remainder >= 5)
	        	{
        			radius_index = (radius_index+1) % numRadii;
	        	}
	        	LinkedList<Neuron> radius  = radii.get(radius_index);
	        		        	
	        	for (Neuron neuron : radius) // Look for the first neuron that "sees" the event
	        	{
	        		Point2D.Float loc = neuron.getLocation();
	        		int field =  neuron.getReceptiveFieldSize();
	        		Point2D.Float vector = new Point2D.Float(pe.x-loc.x,pe.y-loc.y);
	        		double dist = Math.sqrt((double)((vector.x)*(vector.x)+(vector.y)*(vector.y)));
	        		if(dist <= field)
	        		{
	        			/** I'm dealing with forward motion only, so any events that are located "in front of" the neuron, 
	        			 * in respect to the nodal point, are added to the next neuron
	        			 */
	        			if((loc.x >= 64 && (vector.x <= 0 || (vector.x > 0 && vector.x < receptiveFieldSizePixels))) || 
	        					(loc.x < 64 && (vector.x > 0 || (vector.x <= 0 && vector.x > -receptiveFieldSizePixels))))
	        			{
	        				neuron.addEvent(pe, 1.0f);
		        			break;
	        			}
	        			
	        		}
	        	}
	        }
	        updatedNeurons = maybeCallUpdateObservers(in, lastTime);
		}
		return out;
	}
	
	public void assignToNeuronLine(ArrayList<Neuron> neuronLine, PolarityEvent pe) {
		for(Neuron neuron : neuronLine)
		{
			Point2D.Float location = neuron.getLocation();
			Point2D.Float vector = new Point2D.Float(pe.x-location.x,pe.y-location.y);
    		double dist = Math.sqrt((double)((vector.x)*(vector.x)+(vector.y)*(vector.y)));
    		if(dist <= neuron.getReceptiveFieldSize())
    			neuron.addEvent(pe, 0.5f);
		}
		
	}

	/**
	 * Create the two neuron lines.
	 */
	private void initNeuronLines()
	{
		int ySpreadOut = 5;
		int yStart = 10;
		int yEnd = 110;
		int leftX = 3;
		int rightX = 125;
		for(int yCoord = yStart; yCoord <= yEnd; yCoord += ySpreadOut)
		{
			Neuron leftNeuron = new Neuron(0, null, 0, new Point2D.Float(leftX, yCoord), 0);
			Neuron rightNeuron = new Neuron(0, null, 0, new Point2D.Float(rightX, yCoord), 0);
			neuronLineLeft.add(leftNeuron);
			neuronLineRight.add(rightNeuron);
			leftNeuron.setReceptiveFieldSize(ySpreadOut*2);
			rightNeuron.setReceptiveFieldSize(ySpreadOut*2);
		}
	}
	  
	@Override
	/**
	 * Initialize the filter. Creates the network and resets the values
	 */
	synchronized public void initFilter() {
		neurons.clear();
		radii.clear();
		if(controller != null)
			controller.reset();
		if(data != null)
			data.reset();
		if(neuronLineLeft.isEmpty() || neuronLineRight.isEmpty())
		{
			initNeuronLines();
		}
		if(this.angle == 0)
			resetFilter();
		
		if(LEVEL == FilterLevel.DEBUG) // Only one radius in DEBUG mode
		{
			numberRadii = 1;
			for(int radius = 0;radius<numberRadii;radius++)
			{
				LinkedList<Neuron> rNeurons = new LinkedList<Neuron>();
				radii.add(radius,rNeurons);
				double cos, sin = 0f;
				double angle = (radius == 0) ? Math.toRadians(this.angle) : Math.toRadians(180-this.angle);
				cos = Math.cos(angle);
				sin = Math.sin(angle);
				for(int numNeuron=1;numNeuron <= neuronsOnRadius;numNeuron++)
	        	{
	        		Point2D.Float neuronLocation = computeNeuronLocation(numNeuron,cos,sin);
	        		if(neuronLocation.x <= 128 && neuronLocation.x >= 0 && neuronLocation.y >= 0 && neuronLocation.y <= 128)
	        		{
	        			int cellNumber = radius*(neuronsOnRadius-1) + numNeuron-1;
	        			Neuron neuron = new Neuron(cellNumber, rNeurons, numNeuron, neuronLocation,radius);
	        			if(numNeuron != 1)
	        			{
	        				neuron.setLowNeuron(neurons.get(cellNumber-1));
	        				neurons.get(cellNumber-1).setHighNeuron(neuron);
	        			}
	        			rNeurons.add(neuron);
	        			neurons.add(cellNumber,neuron);
	        		}
	            	
	        	}
			}
			
		}
		else
		{
			numberRadii = (int)(360/this.angle);
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
		    			rNeurons.add(neuron);
		    			neurons.add(cellNumber,neuron);
		    		}
		        	
		    	}
		    }
		}
		for(Neuron neuron: neurons) // Second run: define the receptive field of each neuron
		{
			Point2D.Float thisLocation = neuron.getLocation();
			try
			{
				Point2D.Float neighborLocation = neuron.getLowNeuron().getLocation(); 
				double dist = Math.ceil(Math.hypot(thisLocation.x-neighborLocation.x, thisLocation.y - neighborLocation.y));
				int receptiveField = (int)(dist/2);
				if(receptiveField < 2) // The minimum receptive field is controller by receptiveFieldSizePixels
					receptiveField = 2;
				/*if(receptiveField < 2)
					receptiveField = 2;*/
				neuron.setReceptiveFieldSize(receptiveField);
			}
			catch (NullPointerException exc)
			{
				neuron.setReceptiveFieldSize(receptiveFieldSizePixels);
			}
			
			
		}
	}
	
	/**
	 * For optimization. Sets the next value for the optimized variable
	 * @param optimCase: the variable that is considered
	 * @param value: the new value
	 * @param shouldIncrease: is it an increase by value, or simply a new value?
	 */
	public void setOptimizedValue(int optimCase, float value, boolean shouldIncrease)
	{
		switch(optimCase)
		{
		case 0:
			setArrivalThreshold(shouldIncrease?arrivalThreshold + (int)value: (int)value);
			break;
		case 1:
			setCompleteOnConfirmedSignals(shouldIncrease?completeOnConfirmedSignals+(int)value:(int)value);
			break;
		case 2:
			setConclusionDepth(shouldIncrease?conclusionDepth+(int)value:(int)value);
			break;
		case 3:
			setConfirmThreshold(shouldIncrease?confirmThreshold+(int)value:(int)value);
			break;
		case 4:
			setExpectedInitialSignalVelocity(shouldIncrease?expectedInitialSignalVelocity+value:value);
			break;
		case 5:
			setNeuronsOnRadius(shouldIncrease?neuronsOnRadius+(int)value:(int)value);
			break;
		case 6:
			setVelocityErrorThreshold(shouldIncrease?velocityErrorThreshold+value:value);
			break;
		}
	}
	
	/**
	 * Get the value of the optimized variable
	 * @param optimCase: the variable that is currently being optimized
	 * @return the current value of the variable
	 */
	public float getOptimizedValue(int optimCase)
	{
		switch(optimCase)
		{
			case 0:
				return arrivalThreshold; 
			case 1:
				return completeOnConfirmedSignals;
			case 2:
				return conclusionDepth;
			case 3:
				return confirmThreshold;
			case 4:
				return expectedInitialSignalVelocity;
			case 5:
				return neuronsOnRadius;
			case 6:
				return velocityErrorThreshold;
			default:
				return 0;
		}
	}
	
	
	/**
	 * Set the initial values for evaluation
	 */
	public void initializeEvaluationVariables()
	{
		/**
		setArrivalThreshold(131000);
		setCompleteOnConfirmedSignals(10);
		setConclusionDepth(8);
		setConfirmThreshold(DEFAULT_CONFIRMTHRESH);
		setExpectedInitialSignalVelocity(28);
		setVelocityErrorThreshold(0.5f);
		setMPThreshold(3);
		setNeuronsOnRadius(10);*/

		setArrivalThreshold(100000);
		setCompleteOnConfirmedSignals(10);
		setConclusionDepth(8);
		setConfirmThreshold(2);
		setExpectedInitialSignalVelocity(28);
		setVelocityErrorThreshold(0.5f);
		setMPThreshold(2);
		setNeuronsOnRadius(10);
	}
	
	/**
	 * For evaluation. Set the 
	 * @param evalCase
	 * @param value
	 */
	public void setEvaluationValue(int evalCase, float value)
	{
		switch(evaluationCase)
		{
		case 0:
			setMPThreshold((int)value);
			break;
		case 1:
			setCompleteOnConfirmedSignals((int)value);
			break;
		case 2:
			setConclusionDepth((int)value);
			break;
		case 3:
			setConfirmThreshold((int)value);
			break;
		case 4:
			setExpectedInitialSignalVelocity(value);
			break;
		case 5:
			setNeuronsOnRadius((int)value);
			break;
		case 6:
			setVelocityErrorThreshold(value);
			break;
		case 7:
			setArrivalThreshold((int)value);
			break;
		}
	}
	
	/** Computes the average number of signals, confirmed signals, the average ratio of confirmed vs created signals.
	 * Gets the conclusion set an computes the % of correct conclusions
	 */
	public void evaluateResultsAndPrint()
	{
		float avgSignals,avgConfirmedSignals,avgFiringNeurons,successRatio,signalRatio, leftRightRatio;
		int[] conclusions;
		avgSignals = (float)getNumberCreatedSignals() / (float)iterationCounter;
		avgConfirmedSignals = (float)getNumberConfirmedSignals() / (float)iterationCounter;
		avgFiringNeurons = (float)getNumberFiringNeurons() / (float)iterationCounter;
		signalRatio = avgConfirmedSignals / avgSignals;
		leftRightRatio = data.getSignalRatio(this.ROBOT_START_POSITION);
		conclusions = data.getConclusionsAndReset();
		float totalNumberConclusions = (float)(conclusions[0]+conclusions[1]+conclusions[2]);  
		successRatio = (totalNumberConclusions > 0)?(float)conclusions[ROBOT_START_POSITION] / totalNumberConclusions:0;
		try 
		{
			evaluationBuffWriter.write("|"+evaluationCase+"|"+evaluationValue+"|"+avgFiringNeurons+"|"
								+avgConfirmedSignals+"|"+avgSignals+"|"+successRatio+"|"+leftRightRatio+"|\n");
		} 
		catch (IOException exception) 
		{
			exception.printStackTrace();
		}
		System.out.println("|     "+evaluationCase+"     |     "+evaluationValue+"     |     "+avgFiringNeurons+"     |     "
				+avgConfirmedSignals+"     |     "+avgSignals+"     |     "+successRatio+"     |     "+leftRightRatio+"     |\n");
		
		if(evaluationCase == 5)
		{
			int totalReceptiveField = 0;
			float avgReceptiveField = 0;
			for(Neuron lifNeuron : neurons)
			{
				totalReceptiveField += lifNeuron.getReceptiveFieldSize();
			}
			avgReceptiveField = (float)totalReceptiveField / (float)neurons.size();
			System.out.println("AVG RECEPTIVE FIELD: " + avgReceptiveField);
		}
		
	}
	
	/**
	 * Do this on every filter reset to obtain intermediate evaluation results
	 */
	public void performEvaluationStep()
	{
		if(isJustStarted)
		{
			System.out.println("Evaluation ON\nRobot position is: " + ROBOT_START_POSITION);
			try 
			{
				evaluationBuffWriter.write("|     CASE     |     VALUE     |     AVG_FIRE     |     AVG_CONF     |     AVG_CREATE     |     CORR%     |     RATIO     |\n");
			} 
			catch (IOException exception) 
			{
				exception.printStackTrace();
			}
			System.out.println("|     CASE     |     VALUE     |     AVG_FIRE     |     AVG_CONF     |     AVG_CREATE     |     CORR%     |     RATIO     |");
			evaluationCase = 5; // Here, I can set the initial (and maybe only) evaluation case	
			evaluationErrorRatio = 0;
			evaluationValue = 0;
			numberEvents = 0;
			numberConfirmedSignals = 0;
			numberCreatedSignals = 0;
			evaluationValueChange = 0;
			evaluationValueFinish = 0;
			numberFiringNeurons = 0;   
			ratioLeftRightSignals = 0;
		}
		if(isSwitchingCase)
		{
			isSwitchingCase = false;
			if(isJustStarted)
			{
				/** Initialize stuff */
				isJustStarted = false;
			}
			else
			{
				/** DO final computation */
				evaluateResultsAndPrint();
				if(EVAL_SINGLE_CASE)
					evaluationCase = 8; // Break;
				else
					evaluationCase++;
				
			}
			initializeEvaluationVariables(); // Reset to initial evaluation values
			switch(evaluationCase)
			{
				case 0: // MPThreshold
					evaluationValue = 1;
					setEvaluationValue(evaluationCase, evaluationValue);
					evaluationValueChange = 1;
					evaluationValueFinish = 10;
					break;
				case 1: // completeOnConfirmedSignals
					evaluationValue = 1;
					setEvaluationValue(evaluationCase, evaluationValue);
					evaluationValueChange = 2;
					evaluationValueFinish = 40;
					break;
				case 2: // Conclusion threshold
					evaluationValue = 1;
					setEvaluationValue(evaluationCase, evaluationValue);
					evaluationValueChange = 2;
					evaluationValueFinish = 12;
					break;
				case 3: // Confirm threshold
					evaluationValue = 1;
					setEvaluationValue(evaluationCase, evaluationValue);
					evaluationValueChange = 1;
					evaluationValueFinish = 5;
					break;
				case 4: // Initial expected velocity
					evaluationValue = 10f;
					setEvaluationValue(evaluationCase, evaluationValue);
					evaluationValueChange = 10f;
					evaluationValueFinish = 200f;
					break;
				case 5: // Neurons per radius: Not used in the formal analysis, but check anyway. 10 neurons per radius is great
					evaluationValue = 4;
					setEvaluationValue(evaluationCase, evaluationValue);
					evaluationValueChange = 4;
					evaluationValueFinish = 40;
					break;
				case 6: // Velocity Error threshold
					evaluationValue = 0.1f;
					setEvaluationValue(evaluationCase, evaluationValue);
					evaluationValueChange = 0.1f;
					evaluationValueFinish = 4f;
					break;
				case 7: // Arrival Threshold
					evaluationValue = 20000;
					setEvaluationValue(evaluationCase, evaluationValue);
					evaluationValueChange = 27000;
					evaluationValueFinish = 600000;
					break;	
				case 8:
					System.out.println("Evaluation done. Quitting...");
					resetValues();
            		System.exit(0);
            		break;
			}
			
		}
		else
		{
			/**
			 * The main task is to check whether the variable has reached its upper bound, and change the evaluation case
			 * Other tasks go here.
			 */
			switch(ADDITIONAL_EVALUATION_TASKS)
			{
				case 0:
					evaluateResultsAndPrint();	
					evaluationValue += evaluationValueChange;
					setEvaluationValue(evaluationCase,	evaluationValue);
					if(evaluationValue + evaluationValueChange > evaluationValueFinish) // If next increase exceeds the finish value, switch cases
					{
						isSwitchingCase = true;
					}
					
					
					
					
					break;
				case 1: 
					/** Only evaluate light */
					float avgEvents = (float)getNumberEvents() / (float)iterationCounter ;
					float avgSignals = (float)getNumberCreatedSignals() / (float)iterationCounter;
					float avgConfirmedSignals = (float)getNumberConfirmedSignals() / (float)iterationCounter;
					float signalRatio = avgConfirmedSignals / avgSignals;
					int[] conclusions = data.getConclusionsAndReset();
					float successRatio = (float)conclusions[ROBOT_START_POSITION] / (float)(conclusions[0]+conclusions[1]+conclusions[2]);
					System.out.println("Light evaluation complete\nNumber of Iterations: "+iterationCounter+
							"\nAverage Events: " + avgEvents+"\nAverage Signals: "+ avgSignals +"\nAverage Confirmed Signals: "+
							avgConfirmedSignals+"\nSignal Ratio: " + signalRatio + "\nSuccess Ratio: " + successRatio);
					break;
				
			}
			
		}
	}

	/**
	 * Initialize the optimization variables
	 * @param optimCase: set this to a specific variable to only initialize its value. Otherwise, initialize all
	 */
	public void initializeOptimizationVariables(int optimCase)
	{
		if(optimCase == -1) // Initialize all variables
		{
			setArrivalThreshold(25000);
			setCompleteOnConfirmedSignals(3);
			setConclusionDepth(2);
			setConfirmThreshold(2);
			setExpectedInitialSignalVelocity(6f);
			setVelocityErrorThreshold(0.4f);
		}
		else
		{
			switch(optimCase)
			{
				case 0:
					optimizationValue = 25000;
					setArrivalThreshold(25000);
					break;
				case 1:
					setCompleteOnConfirmedSignals(1);
					break;
				case 2:
					setConclusionDepth(1);
					break;
				case 3:
					setConfirmThreshold(1);
					break;
				case 4:
					setExpectedInitialSignalVelocity(4f);
					break;
				case 5:
					setNeuronsOnRadius(DEFAULT_NEURONS_RADIUS);
					break;
				case 6:
					setVelocityErrorThreshold(0.2f);
					break;
			}
		}
	}
	
	
	/**
	 * Check if score is optimum from the optimization
	 * @param score: the current score
	 * @param optimValue: the value for the variable that is optimized
	 */
	public void computeBestScore(float score, float optimValue)
	{
		if(score >= optimumScore)
		{
			optimumScore = score;
			System.out.println("New best score: " + score);
			setOptimizedValue(optimCase,optimValue,false);
			bestCombination = new OptimizationCombination(completeOnConfirmedSignals, conclusionDepth, confirmThreshold, 
					neuronsOnRadius, arrivalThreshold, expectedInitialSignalVelocity, velocityErrorThreshold, optimumScore);			
		}
	}
	
	
	private void checkScore() {
		
		int[] conclusions = data.getConclusionsAndReset();
		int totalCountConclusions = conclusions[0] + conclusions[1] + conclusions[2];
		float percentCorrect = totalCountConclusions != 0?(float) conclusions[ROBOT_START_POSITION] / (float)totalCountConclusions:0;
		computeBestScore(percentCorrect,optimizationValue);
			
	}
	
	

	public void performOptimizationStep()
	{
		if(isJustStarted)
		{
			System.out.println("Optimize ON\nRobot position is: " + ROBOT_START_POSITION);
			optimMap = new Hashtable<java.lang.Float, java.lang.Float>();
			optimCase = 0;
			optimumScore = 0;
			optimizationValue = 0;
			optimValueChange = 0;
			initializeOptimizationVariables(-1);
			
		}
		if(isSwitchingCase)
		{
			isSwitchingCase = false;
			if(isJustStarted)
			{
				/** Initialize stuff */
				isJustStarted = false;
				optimizationValue = 25000;
				optimValueChange = 15000;
				optimFinish = 200000;
			}
			else
			{
				checkScore();
				initializeOptimizationVariables(optimCase); // Reset optimized variable (case 0) to initial value
				optimCase++;
			}
			if(optimCase == 1)
			{
				float currValue = getOptimizedValue(optimCase);
				if (currValue > 20) // 20 is finish value
				{
					System.out.println("Case " + optimCase+" reached finish value. Resetting and increasing case value");
					// Reset value, increase case
					initializeOptimizationVariables(optimCase);
					optimCase++;					
				}
				else
				{
					float newValue = currValue + 3;
					System.out.println("Case " + optimCase+" . Increasing value. New value is: " + newValue);
					setOptimizedValue(optimCase, newValue, false);
					optimCase = 0;
				}
			}
			if(optimCase == 2)
			{
				float currValue = getOptimizedValue(optimCase);
				if (currValue > 12) 
				{
					System.out.println("Case " + optimCase+" reached finish value. Resetting and increasing case value");
					// Reset value, increase case
					initializeOptimizationVariables(optimCase);
					optimCase++;					
				}
				else
				{
					float newValue = currValue + 2;
					System.out.println("Case " + optimCase+" . Increasing value. New value is: " + newValue);
					setOptimizedValue(optimCase, newValue, false);
					optimCase = 0;
				}
			}
			if(optimCase == 3)
			{
				float currValue = getOptimizedValue(optimCase);
				if (currValue > 6) 
				{
					System.out.println("Case " + optimCase+" reached finish value. Resetting and increasing case value");
					// Reset value, increase case
					initializeOptimizationVariables(optimCase);
					optimCase++;					
				}
				else
				{
					float newValue = currValue + 1;
					System.out.println("Case " + optimCase+" . Increasing value. New value is: " + newValue);
					setOptimizedValue(optimCase, newValue, false);
					optimCase = 0;
				}
			}
			if(optimCase == 4)
			{
				float currValue = getOptimizedValue(optimCase);
				if (currValue > 80) 
				{
					System.out.println("Case " + optimCase+" reached finish value. Resetting and increasing case value");
					// Reset value, increase case
					initializeOptimizationVariables(optimCase);
					optimCase++;					
				}
				else
				{
					float newValue = currValue + 6f;
					System.out.println("Case " + optimCase+" . Increasing value. New value is: " + newValue);
					setOptimizedValue(optimCase, newValue, false);
					optimCase = 0;
				}
			}
			if(optimCase == 5)
			{
				float currValue = getOptimizedValue(optimCase);
				if (currValue > 60) 
				{
					System.out.println("Case " + optimCase+" reached finish value. Resetting and increasing case value");
					// Reset value, increase case
					initializeOptimizationVariables(optimCase);
					optimCase++;					
				}
				else
				{
					float newValue = currValue + 6;
					System.out.println("Case " + optimCase+" . Increasing value. New value is: " + newValue);
					setOptimizedValue(optimCase, newValue, false);
					optimCase = 0;
				}
			}
			if(optimCase == 6)
			{
				float currValue = getOptimizedValue(optimCase);
				if (currValue > 4f) 
				{
					System.out.println("Case " + optimCase+" reached finish value. Resetting and increasing case value");
					// Reset value, increase case
					initializeOptimizationVariables(optimCase);
					optimCase++;					
				}
				else
				{
					float newValue = currValue + 0.6f;
					System.out.println("Case " + optimCase+" . Increasing value. New value is: " + newValue);
					setOptimizedValue(optimCase, newValue, false);
					optimCase = 0;
				}
			}
			if(optimCase == 7)
			{
				// Finish
				System.out.println("Optimization done. Quitting...");
				System.out.println("|   NUMBER CONFIRMED SIGNALS PER CONCLUSION   |   CONCLUSION DEPTH   |   CONFIRMATION THRESHOLD   " +
						"|   NEURONS ON RADIUS   |   SIGNAL ARRIVAL THRESHOLD   |   EXPECTED INITIAL SIGNAL VELOCITY   |   VELOCITY ERROR THRESHOLD   |   SCORE   |");
				System.out.println("|   "+bestCombination.getCompleteOnConfirmedSignals()+"   |   "+bestCombination.getConclusionDepth()+"   " +
									"|   "+bestCombination.getConfirmThreshold()+"   |   "+bestCombination.getNeuronsOnRadius()+"   " +
									"|   "+bestCombination.getSignalArrivalThreshold()+"   |   "+bestCombination.getExpectedInitialSignalVelocity()+"   " +
									"|   "+bestCombination.getVelocityErrorThreshold()+"   |   "+bestCombination.getScore()+"   |");
				try 
				{
					optimBuffWriter.write("|   NUMBER CONFIRMED SIGNALS PER CONCLUSION   |   CONCLUSION DEPTH   |   CONFIRMATION THRESHOLD   " +
							"|   NEURONS ON RADIUS   |   SIGNAL ARRIVAL THRESHOLD   |   EXPECTED INITIAL SIGNAL VELOCITY   |   VELOCITY ERROR THRESHOLD   |   SCORE   |");
					optimBuffWriter.write("|   "+bestCombination.getCompleteOnConfirmedSignals()+"   |   "+bestCombination.getConclusionDepth()+"   " +
							"|   "+bestCombination.getConfirmThreshold()+"   |   "+bestCombination.getNeuronsOnRadius()+"   " +
							"|   "+bestCombination.getSignalArrivalThreshold()+"   |   "+bestCombination.getExpectedInitialSignalVelocity()+"   " +
							"|   "+bestCombination.getVelocityErrorThreshold()+"   |   "+bestCombination.getScore()+"   |");
				} 
				catch (IOException exception) 
				{
					exception.printStackTrace();
				}
				resetValues();
        		System.exit(0);
			}				
		}
		else
		{
			checkScore();
			
			optimizationValue += optimValueChange;
			
			System.out.println("New value is: "+ optimizationValue);
			setOptimizedValue(optimCase, optimizationValue, false);
			if(optimizationValue + optimValueChange > optimFinish)
				isSwitchingCase = true;
			
		}
	}
	
	
	@Override
	public void resetFilter() {
		
		for (Neuron n : neurons) {
            n.reset(true);
        }
		if(controller != null)
			controller.reset();
		if(data != null)
			data.reset();
		opticalFlowVectors.clear();
		memoryUpdates.clear();
        lastTime = 0;
        firingNeurons.clear();
           	
    	if(LEVEL == FilterLevel.EVALUATION)
    	{
    		performEvaluationStep();
    		
    	}
    	else if (LEVEL == FilterLevel.OPTIMIZE)
    	{
    		performOptimizationStep();
    	}
    	    		
    	/**
    	 * Not very appropriate to initialize the filter twice before actually running
    	 */
    	//else
    		//resetValues();//initFilter();
    	iterationCounter = 0;
	}
	
	/**
	 * Check the neuron line. If a certain number of neurons have fired on either side, than that particular wall is still visible
	 * Used for correcting the robot's position
	 * @param now the current time
	 */
	private void checkLineNeurons(int now)
	{
		int numberFiredLeft = 0;
		int numberFiredRight = 0;
		int timeSinceSupport;
		for(int index = 0; index < neuronLineLeft.size(); index++)
		{
			Neuron leftNeuron = neuronLineLeft.get(index);
			Neuron rightNeuron = neuronLineRight.get(index);
			timeSinceSupport = now - leftNeuron.getLastEventTimestamp();
			if(timeSinceSupport > neuronStaleReset)
				leftNeuron.reset(false);
			timeSinceSupport = now - rightNeuron.getLastEventTimestamp();
			if(timeSinceSupport > neuronStaleReset)
				rightNeuron.reset(false);
			if(leftNeuron.isAboveThreshold(lastTime))
			{
				numberFiredLeft++;
			}
			if(rightNeuron.isAboveThreshold(lastTime))
				numberFiredRight++;
		}
		boolean leftCheck = (numberFiredLeft >= 0.6 * neuronLineLeft.size())?
							true:
							false;
		boolean rightCheck = (numberFiredRight >= 0.6 * neuronLineRight.size())?
							true:
							false;
		/*if(leftCheck)
			System.out.println("Left neuron line check passes");
		else
			System.out.println("Left neuron line check fails");
		if(rightCheck)
			System.out.println("Right neuron line check passes");
		else
			System.out.println("Right neuron line check fails");*/
		if(controller.isCorrectionPhase())
		{
			data.setLeftLineCheck(leftCheck);
			data.setRightLineCheck(rightCheck);
			data.setChecksReady(true);
		}
	}
		    
    /**
     * Updates all neurons at time t.
     *
     * Checks if the neuron is firing.
     * Checks neuron's memory for stored signals
     * Updates neuron memory
     * If enough signals have been confirmed, switches semaphore to allow controller to process data
     *
     * @param t
     */
    public synchronized void updateNeurons(int t) {
    	firingNeurons.clear();
    	
    	checkLineNeurons(t);
    	
    	if (!neurons.isEmpty()) {
            int timeSinceSupport;
            for(Neuron neuron: neurons)
            {
            	try {
                    // reset stale neurons
            		/*timeSinceSupport = t - neuron.getLastEventTimestamp();
                    if (timeSinceSupport > neuronStaleReset)
                    {
                    	if(LEVEL == FilterLevel.DEBUG && neuron.isFired())
                    	{
                    		System.out.println("neuron.reset():Radius:"+neuron.getRadiusNum()+";Neuron: "+ neuron.getCellNumber()+";Time: " + t+"; lastEventTimestamp:" + neuron.getLastEventTimestamp());
                    	}
                    	//neuron.reset(false );
                    	
                    }*/
                    if(neuron.isAboveThreshold(lastTime))
                    {
                    	firingNeurons.add(neuron.getCellNumber());
                    	numberFiringNeurons++;
                    }
                    neuron.checkMemory(t);
                } catch (java.util.ConcurrentModificationException e) {
                    // this is in case neuron list is modified by real time filter during updating neurons
                    initFilter();
                    log.warning(e.getMessage());
                } 
            }           
                        
            for(Neuron neuron: memoryUpdates)
            {
            	neuron.updateMemory();
            	neuron.resetEdgeCounters();
            	neuron.resetEdgePosition();
            }
            memoryUpdates.clear();
            if(data != null)
            {
            	if(signalCounter >= completeOnConfirmedSignals)
            	{
            		data.setDataComplete(true);
            		signalCounter = 0;
            	}           	         	
            }
            	
        }
    }
    
    /**
     * Sets the signal velocity error threshold
     * @param velocityErrorThreshold the new value
     */
         
    public void setVelocityErrorThreshold(float velocityErrorThreshold)
    {
    	this.velocityErrorThreshold = velocityErrorThreshold;
    	getPrefs().putFloat("ImageReceiver4.velocityErrorThreshold", velocityErrorThreshold);
        if(controller != null)
        	controller.setVelocityErrorThreshold(velocityErrorThreshold);
    }
    
    
    /**
     * Get the velocity error threshold
     * @return velocityErrorThreshold The current velocity threshold
     */
    public float getVelocityErrorThreshold()
    {
    	return velocityErrorThreshold;
    }
    
    /**
     * Set the number of neurons per radius. Initializes the filter to update the network
     * @param neuronsOnRadius the number of neurons
     */
    
    synchronized public void setNeuronsOnRadius(int neuronsOnRadius) {
        if(neuronsOnRadius > 2)
        {
        	this.neuronsOnRadius = neuronsOnRadius;
            getPrefs().putInt("ImageReceiver4.neuronsOnRadius", neuronsOnRadius);
            initFilter();
        }
    	                
    }
    
	
    /**
     * Increases the counter used for completeOnConfirmedSignals
     */
    public void increaseSignalCounter() {
		signalCounter++;		
	}
	
	
    /**
     * Sets the conclusion depth
     * @param conclusionDepth the new value
     */
    public void setConclusionDepth(int conclusionDepth)
    {
    	this.conclusionDepth = conclusionDepth;
    	getPrefs().putInt("ImageReceiver.conclusionDepth", conclusionDepth);
    	if(controller != null)
    		controller.setDefaultConclusionDepth(conclusionDepth);
    }

	/**
	 * Returns the total number of Events.
	 * Will reset the count before returning it
	 * @return
	 */
	public int getNumberEvents()
	{
		int num = this.numberEvents;
		this.numberEvents = 0;
		return num;
	}
	
	/**
	 * Returns the number of confirmed signals
	 * Will reset the count before returning it
	 */
	public int getNumberConfirmedSignals()
	{
		int num = this.numberConfirmedSignals;
		this.numberConfirmedSignals = 0;
		return num;
	}
	
	/**
	 * Returns the number of created signals
	 * Will reset the count before returning it
	 * @return
	 */
	public int getNumberCreatedSignals()
	{
		int num = this.numberCreatedSignals;
		this.numberCreatedSignals = 0;
		return num;
	}

	
	/**
	 * Gets the total number of firing neurons
	 * Will reset the count before returning it
	 * @return count
	 */
	public int getNumberFiringNeurons()
	{
		int count = this.numberFiringNeurons;
		this.numberFiringNeurons = 0;
		return count;
	}
	
}
