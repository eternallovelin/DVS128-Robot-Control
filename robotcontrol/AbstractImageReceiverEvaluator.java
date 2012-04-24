package jaer.myjaer.robotcontrol;

import jaer.myjaer.neuron.Neuron;

import java.awt.geom.Point2D;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;

import net.sf.jaer.chip.AEChip;
import net.sf.jaer.event.EventPacket;
import net.sf.jaer.event.PolarityEvent;

public abstract class AbstractImageReceiverEvaluator extends AbstractImageReceiver{


	// Evaluation output
    /** File writers for dumping data. Do this until I write a proper log handler, NO TIME NOW */
    protected String signalsMeasurementsFlname = "/home/vanxa/workspace/Project/evaluation/signal_measurements.txt";
    protected String ofAccuracyPercentageFlname = "/home/vanxa/workspace/Project/evaluation/vector_measurements.txt";
    protected String ofVelocityMeasurementFlname = "/home/vanxa/workspace/Project/evaluation/velocity_measurements.txt";

    protected FileWriter confirmedSignalsWriter;
    protected FileWriter opticalAccuracyPercentageWriter;
    protected FileWriter velocityMeasurementWriter;
    
    protected BufferedWriter confirmedSignalsBuffWriter;
    protected BufferedWriter opticalAccuracyPercentageBuffWriter;    
    protected BufferedWriter velocityMeasurementBuffWriter;
    
    // Evaluation variables
    protected int numberEvents = 0;
    protected int totalSumEvents = 0;
    protected int totalSumConfirmedSignals = 0;
    protected int totalSumSignals = 0;
 	protected int totalSumFiringNeurons = 0;
 	protected int iterationCounter = 0;
 	protected boolean isSwitchingCase = false;
 	protected boolean isJustStarted = true;
 	protected float avgVelocityLeftIteration = 0;
 	protected float avgVelocityRightIteration = 0;
 	protected float totSumVelocityLeft = 0;
 	protected float totSumVelocityRight = 0;
 	protected float avgVelocityDifference = 0;
 	protected int countSignalsLeft = 0;
 	protected int countSignalsRight = 0;
 	
 	/** Total number of signals per processing iteration */
 	protected HashSet<Integer> numberSignals = new HashSet<Integer>();
 	
 	/** Total number of confirmed signals per processing iteration */
 	protected ArrayList<Integer> numberConfirmedSignals = new ArrayList<Integer>();
	
	
	/**
	 * Evaluation cases:
	 * TODO: review
	 * 0 Static: compute number of signals, number of confirmed signals and number of firing neurons without changing variables
	 * 1 Membrane Potential threshold: compute averages while adjusting MPThresh: 2->9; +1
	 * 2 Angle: 8->45; +2
	 * 3 Arriving time threshold: 16000 -> 60000; +4000
	 * 4 Confirm threhold: 3-> 10; +1
	 * 5 Expected initial velocity: 2->10;+1
	 * 6 Neurons per radius: 10->36; +2
	 * 7 Priority threshold: 2->8; +1
	 * 8 Receptive field size: 6->14; +1
	 * 9 Signal time threshold: 16000 -> 60000; +4000
	 * 10 Exit
	 */
	protected int evaluationCase = 1;
	protected int[] evaluationData = new int[3];
	protected String evaluationCaseVarName;	
	 	
	
	public AbstractImageReceiverEvaluator(AEChip chip) {
		super(chip);
		if(LEVEL == FilterLevel.EVALUATION)
        {
        	System.out.println("EVALUATION: ON");
        	try 
            {
    			confirmedSignalsWriter = new FileWriter(signalsMeasurementsFlname);
    			opticalAccuracyPercentageWriter = new FileWriter(ofAccuracyPercentageFlname);
    			velocityMeasurementWriter = new FileWriter(ofVelocityMeasurementFlname);
    		} 
            catch (IOException e) 
            {
    			e.printStackTrace();
    		}
            confirmedSignalsBuffWriter = new BufferedWriter(confirmedSignalsWriter);
            opticalAccuracyPercentageBuffWriter = new BufferedWriter(opticalAccuracyPercentageWriter);
            velocityMeasurementBuffWriter = new BufferedWriter(velocityMeasurementWriter);
              
            DEFAULT_NEURONS_RADIUS = 12;
            setNeuronsOnRadius(DEFAULT_NEURONS_RADIUS);
        }
		
	}
	
	
	
}
