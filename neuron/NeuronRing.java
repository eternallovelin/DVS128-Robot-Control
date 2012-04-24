package jaer.myjaer.neuron;

import java.util.LinkedList;

import net.sf.jaer.event.PolarityEvent.Polarity;

public class NeuronRing extends AbstractNeuronRing {
	
	private int index;
	private Polarity lastPolarity;
	
	public NeuronRing(int index)
	{
		super();
		this.index = index;
	}
	
	public int getIndex()
	{
		return index;
	}
	
	public boolean isFired(int ringFireThreshold/*, int time*/) {
		int count = 0;
		//int polarityOnCounter = 0;
		//int polarityOffCounter = 0;
		for(Neuron neuron : neurons)
		{
			if(neuron.isFired())
			{
				count++;
				/*if(neuron.computePolarity() == 1)
					polarityOnCounter++;
				else
					polarityOffCounter++;*/
			}
				
		}
		
		tempFired = (count>ringFireThreshold/100f*neurons.size())?
					true:
					false;
		/*if(tempFired)
		{
			tempLastTimeFired = time;
			tempPolarity = (polarityOnCounter<polarityOffCounter) ?
					Polarity.Off : 
					Polarity.On;
			
			if(lastPolarity != tempPolarity)
				lastPolarity = tempPolarity;
			else
			{
				tempFired = false;
			}
		}*/	
		
		return tempFired;
	}
		
}
