/**
 * 
 */
package jaer.myjaer.neuron;

import java.util.LinkedList;

import net.sf.jaer.event.PolarityEvent.Polarity;

/**
 * @author vanxa
 *
 */
public abstract class AbstractNeuronRing {
	
	protected LinkedList<Neuron> neurons;
	protected Polarity polarity;
	protected boolean fired;
	protected int lastTimeFired;
	protected AbstractNeuronRing lowRing;
	protected AbstractNeuronRing highRing;
	protected int numberNeurons;
	
	protected boolean tempFired;
	protected int tempLastTimeFired;
	protected Polarity tempPolarity;


	public AbstractNeuronRing()
	{
		neurons = new LinkedList<Neuron>();
		fired = false;
		tempFired = false;
		lastTimeFired = 0;
		tempLastTimeFired = 0;
		numberNeurons = 0;
	}
	
	public void setLowRing(AbstractNeuronRing lowRing)
	{
		this.lowRing = lowRing;
	}
	
	public void setHighRing(AbstractNeuronRing highRing)
	{
		this.highRing = highRing;
	}
	
	public AbstractNeuronRing getLowRing()
	{
		return lowRing;
	}
	
	public AbstractNeuronRing getHighRing()
	{
		return highRing;
	}
	
	public void addNeuron(Neuron neuron)
	{
		neurons.add(neuron);
		numberNeurons++;
	}
	
	public void setPolarity(Polarity polarity)
	{
		this.polarity = polarity;
	}
	
	public Polarity getPolarity()
	{
		return polarity;
	}
	
	public void setTempPolarity(Polarity polarity)
	{
		this.tempPolarity = polarity;
	}
	
	public Polarity getTempPolarity()
	{
		return tempPolarity;
	}
	
	public void setLastTimeFired(int time)
	{
		this.lastTimeFired = time;
	}
	
	public int getLastTimeFired()
	{
		return lastTimeFired;
	}
	
	public LinkedList<Neuron> getNeurons()
	{
		return neurons;
	}
	
	public void setTempFired(boolean fired)
	{
		this.tempFired = fired;
	}
	
	public boolean getTempFired()
	{
		return tempFired;
	}
	
	public void setTempLastTimeFired(int time)
	{
		this.tempLastTimeFired = time;
	}
	
	public int getTempLastTimeFired()
	{
		return tempLastTimeFired;
	}
	
	public void commitTempData()
	{
		this.fired = tempFired;
		this.lastTimeFired = tempLastTimeFired;
		this.polarity = tempPolarity;
	}
	
	public void reset()
	{
		fired = false;
		lastTimeFired = 0;
		polarity = null;
	}
	
}
