package jaer.myjaer.robotcontrol;

import jaer.myjaer.neuron.*;

import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Float;
import java.util.Hashtable;

public class Data 
{
	private Hashtable<Signal,Point2D.Float> vectors;
	private long receiveTime;
	
	private boolean dataComplete = false;
	
	private volatile boolean cameraIdle = false;
	
	private int[] conclusionSet = new int[3];
	
	private float velocityLeft, velocityRight;
	private int leftCount, rightCount;
	private float leftCountRatio, rightCountRatio;
	
	private boolean checksReady = false;
	private boolean leftLineCheck = false;
	private boolean rightLineCheck = false;
	
	private AbstractImageReceiver filter;
		
	public Data(int numTiers, AbstractImageReceiver filter)
	{
		velocityLeft = 0;
		velocityRight = 0;
		leftCount = 0;
		rightCount = 0;
		receiveTime = 0;
		vectors = new Hashtable<Signal,Point2D.Float>();
		this.filter = filter;
	}
	
	/**
	 * 
	 * @param conclusion: 0: turn left, 1: turn right, 2: move forward
	 */
	public void addConclusion(int conclusion)
	{
		if(conclusion > 2)
			return;
		conclusionSet[conclusion]++;
	}
	
	public int[] getConclusionsAndReset()
	{
		int[] concl = conclusionSet;
		conclusionSet = new int[3];
		return concl;
	}

	public synchronized void setLeftLineCheck(boolean leftLineCheck)
	{
		this.leftLineCheck = leftLineCheck;
	}
	
	public synchronized void setRightLineCheck(boolean rightLineCheck)
	{
		this.rightLineCheck = rightLineCheck;
	}
	
	public synchronized boolean getLeftLineCheck()
	{
		return leftLineCheck;
	}
	
	public synchronized boolean getRightLineCheck()
	{
		return rightLineCheck;
	}
	
	public synchronized void setVectorData(Hashtable<Signal, Point2D.Float> hashtable)
	{
		this.vectors = hashtable;
		receiveTime = System.currentTimeMillis();
	}
	
	public synchronized Hashtable<Signal,Point2D.Float> getVectorData()
	{
		return vectors;
	}
	
		
	public synchronized long getReceiveTime()
	{
		return receiveTime;
	}
	
	public synchronized boolean isCameraIdle()
	{
		return cameraIdle;
	}
	
	public synchronized void setCameraIdle(boolean idle)
	{
		this.cameraIdle = idle;
	}
	
	public void addSignal(Signal signal)
	{
		Point2D.Float location = signal.getLocation();
		Point2D.Float vector = new Point2D.Float(location.x - 64, location.y - 64);
		double length = Math.sqrt(vector.x*vector.x + vector.y*vector.y);
		if(location.x <= 64)
		{
			leftCount++;
			leftCountRatio++;
			velocityLeft = (float) (velocityLeft + (signal.getExpectedVelocity() / length));
		}
		else
		{
			rightCount++;
			rightCountRatio++;
			velocityRight = (float)( velocityRight + (signal.getExpectedVelocity() / length));
		}
		
	}
	
	public synchronized void setDataComplete(boolean isDataComplete)
	{
		this.dataComplete = isDataComplete;
		
	}
	
	public synchronized void reset()
	{
		velocityLeft = 0;
		velocityRight = 0;
		leftCount = 0;
		rightCount = 0;
		
	}
	
	/**
	 * Thanks, PPLS :)
	 * @return
	 */
	public synchronized boolean testAndSetDataComplete()
	{
		boolean flag = dataComplete;
		dataComplete = false;
		return flag;
	}
	
	
	public float getVelocityLeft() {
		float velocity =  (leftCount != 0)?
						velocityLeft/(float)leftCount:
						0;
		velocityLeft = 0;
		return velocity;
	}
	
	public float getVelocityRight()
	{
		float velocity = (rightCount != 0)?
				velocityRight/(float)rightCount:
				0;
		velocityRight = 0;
		return velocity;
	}
	
	public int getLeftCounts()
	{
		int counts = leftCount;
		leftCount = 0;
		return counts;
	}
	
	public int getRightCounts()
	{
		int counts = rightCount;
		rightCount = 0;
		return counts;
	}

	
	public boolean testAndSetChecksReady() {
		boolean flag = checksReady;
		checksReady = false;
		return flag;
	}
	
	public void setChecksReady(boolean flag)
	{
		this.checksReady = flag;
	}

	public float getSignalRatio(int robotStartPosition) {
		float ratio = 0;
		if(robotStartPosition == 0) // Right
		{
			ratio = leftCountRatio!=0?rightCountRatio/leftCountRatio:0;
		}
		else
		{
			ratio = rightCountRatio!=0?leftCountRatio/rightCountRatio:0;
		}			
		
		leftCountRatio = 0;
		rightCountRatio = 0;
		return ratio;
	}

}
