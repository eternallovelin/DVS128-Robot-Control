package jaer.myjaer.robotcontrol;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.BufferedReader;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import javax.swing.Timer;

public class Controller implements Runnable {

	// Control levels
	private static enum ControlLevel { TRANSMIT_DEBUG, CONTROL_DEBUG, OPTIMIZE, NORMAL, EVALUATION };
	private static final ControlLevel LEVEL = ControlLevel.CONTROL_DEBUG;
	// Command types
	public static final int CMD_STATUS = 7;
	public static final int CMD_FORWARD = 1;
	public static final int CMD_BACKWARD = 2;
	public static final int CMD_FLEFT = 3;
	public static final int CMD_FRIGHT = 4;
	public static final int CMD_BLEFT = 5;
	public static final int CMD_BRIGHT = 6;
	public static final int CMD_CANCEL = 0;
	public static final int SHUTDOWN = 0xFFFFFFFF;
	// Default values
	public static final int DEF_MOV_TIME = 4;
	public static final int MAX_SPEED = 1023;//550; 
	public static final int MIN_SPEED = 500;//300;
	public final static int PORT = 3333; //4444;
	
	private AbstractImageReceiver filter;
	
	// Data object
	private Data data;
	private int idleCounter = 0;
	private long idleTimestamp = 0;
	
	/** 
	 * Local conclusion depth
	 */
	private static int CONCLUSION_DEPTH = 5;
	private int localConclusionDepth = 5;
	private int numberConclusions = 0;
	/**
	 * Outcome
	 */
	private float error;
	
	/**
	 * Correction phase
	 */
	private final long POS_CORRECTION_TIMEOUT = 2500;
	private final long FAIL_CHECK_TIMEOUT = POS_CORRECTION_TIMEOUT/2;
	private long mainTimerStartTime = 0;
	private long leftTimerStartTime = 0;
	private long rightTimerStartTime = 0;
	private boolean correctionPhase = false;
	
	
	// Control flag for this thread
	private volatile boolean isRunning;
	
	/**
	 * The last command
	 */
	private int[] lastCommand = new int[3];
	
	/**
	 * The computed velocities
	 */
	private float velocityLeft = 0;
	private float velocityRight = 0;
		
	// Connection settings
	private ServerSocket serverSocket;
	private BufferedReader fromClient;
	private DataOutputStream toClient;
	private Socket clientSocket;
	
	private float currentError;
	private float speedWeight=0;
	
	private boolean isConclusionReady = false;
	private int currentConclusion = 0;
	private Timer timer;
	private int timeout = 10000;
	
	private static float velocityErrorThreshold = 3.0f;
	
	public Controller(Data data, AbstractImageReceiver filter)
	{
		isRunning = true;
		this.data = data;
		timer = new Timer(timeout, handleTimeout);
		this.filter = filter;
	}
	
	@Override
	public void run() {
		if((LEVEL == ControlLevel.NORMAL || LEVEL == ControlLevel.TRANSMIT_DEBUG) && !initResources())
			shutdown();
		while(isThreadRunning())
		{
			if((LEVEL == ControlLevel.NORMAL || LEVEL == ControlLevel.TRANSMIT_DEBUG) && (clientSocket == null || clientSocket.isClosed()))
				connect();
			if(LEVEL == ControlLevel.TRANSMIT_DEBUG)
			{
				AdjustRobotPosition(CMD_FRIGHT, 0.4f);
				while(!isRobotReady(true,5));
			}
			else
			{
				if(data.isCameraIdle() && LEVEL != ControlLevel.OPTIMIZE)
				{
					if(idleCounter == 0)
					{
						idleTimestamp = System.currentTimeMillis();
						idleCounter++;
						continue;
					}
					long currentTime = System.currentTimeMillis();
					if(currentTime - idleTimestamp > 2000) // Idle time, can readjust
					{
						if(LEVEL != ControlLevel.CONTROL_DEBUG)
							System.out.println("Robot is idle");
						command(CMD_FORWARD,1,0,true,true);
						idleCounter = 0;
					}
					else
						continue;
					
				}									
				idleCounter = 0;
				idleTimestamp = 0;
				
				/** Enter the correction phase here: loop until Main timeout or a failcheck timeout occurs */
				if(isCorrectionPhase())
				{
					System.out.println("ENTERING CORRECTION PHASE...");
					mainTimerStartTime = System.currentTimeMillis();
					System.out.println("Main timer started...");
					long currentTime = mainTimerStartTime;
					while(currentTime - mainTimerStartTime < POS_CORRECTION_TIMEOUT)
					{
						currentTime = System.currentTimeMillis();
						if(data.testAndSetChecksReady())
						{
							boolean leftCheck = data.getLeftLineCheck();
							boolean rightCheck = data.getRightLineCheck();
							if(leftTimerStartTime == 0 && !leftCheck)
							{
								if(rightCheck) // Ignore if both checks are false
								{
									leftTimerStartTime = currentTime; 
								}
							}
							else if(leftTimerStartTime != 0 && leftCheck)
							{
								leftTimerStartTime = 0;
							}
							else if(leftTimerStartTime!=0 && currentTime - leftTimerStartTime >= FAIL_CHECK_TIMEOUT)
							{
								// FailCheck timeout. Break and adjust
								System.out.println("FAIL CHECK TIMEOUT: LEFT. MUST READJUST, TURN LEFT");
								command(CMD_BRIGHT, 1f, 1, true, true);
								command(CMD_FORWARD,1f,0,true,false);
								resetCorrectionTimers();
								break;
							}
							
							if(rightTimerStartTime == 0 && !rightCheck)
							{
								if(leftCheck) // Ignore if both checks are false
								{
									rightTimerStartTime = currentTime; 
								}
							}
							else if(rightTimerStartTime != 0 && rightCheck)
							{
								rightTimerStartTime = 0;
							}
							else if(rightTimerStartTime !=0 && currentTime - rightTimerStartTime >= FAIL_CHECK_TIMEOUT)
							{
								// FailCheck timeout. Break and adjust
								System.out.println("FAIL CHECK TIMEOUT: LEFT. MUST READJUST, TURN RIGHT");
								command(CMD_BLEFT, 1f, 1, true, true);
								command(CMD_FORWARD,0.7f,0,true,false);
								resetCorrectionTimers();
								break;
							}
						}
					}
					System.out.println("MAIN TIMEOUT. POSITION IS OK. CONTINUING AS NORMAL");
					resetCorrectionTimers();
					correctionPhase = false;
					
					
				}
				
				
				if(data.testAndSetDataComplete())
				{
					getData();					
					currentError = computeVelocityError();
					error += currentError; // Integrate immediate errors
					boolean controlDebugFlag = (LEVEL == ControlLevel.CONTROL_DEBUG)?true:false;
					if(controlDebugFlag)
					{
						System.out.println("ERROR IS: " + currentError+";till velocity error thresh: "+ (Math.abs(currentError - getVelocityErrorThreshold())));
					}
					numberConclusions++;
					if(numberConclusions > getLocalConclusionDepth()) // Proceed with computing the final outcome
					{
						{
							numberConclusions = 0;
							boolean realFlag = (LEVEL == ControlLevel.NORMAL || LEVEL == ControlLevel.TRANSMIT_DEBUG)?true:false;
							boolean evalFlag = (LEVEL == ControlLevel.EVALUATION)?true:false;
							speedWeight = computeSpeedWeight((float) (Math.abs(error)));
							if(error > getVelocityErrorThreshold())
							{
								if(realFlag)
								{
									System.out.println("MUST TURN RIGHT");
									AdjustRobotPosition(CMD_FRIGHT, speedWeight);
								}										
								if(controlDebugFlag)
								{
									System.out.println("MUST TURN RIGHT. ERROR:" + error);
									
								}
								else if(evalFlag)
									data.addConclusion(1);
							}
							else if(error < 0 && error < -getVelocityErrorThreshold())
							{
								if(realFlag)
								{
									System.out.println("MUST TURN LEFT");
									AdjustRobotPosition(CMD_FLEFT, speedWeight);
								}
									
								if(controlDebugFlag)
								{
									System.out.println("MUST TURN LEFT. ERROR:" + error);
								}
								else if(evalFlag)
									data.addConclusion(0);
							}
							else
							{
								if(realFlag)
								{
									System.out.println("MUST CONTINUE FORWARD");
									command(CMD_FORWARD,1,0,false,false);
								}
								if(controlDebugFlag)
								{
									System.out.println("MUST CONTINUE FORWARD. ERROR:" + error);
									
								}
								else if(evalFlag)
									data.addConclusion(2);
							}
							error = 0;
						}
					}
					
					
				}
			}
			
		}
		System.out.println("Sending shutdown signal to modules...");
		transmit(SHUTDOWN);
		System.out.println("Controller shutting down...");
				
	}
	
	private void resetCorrectionTimers() {
		mainTimerStartTime = 0;
		leftTimerStartTime = 0;
		rightTimerStartTime = 0;
		
	}

	/**
	 * Turning speed weight: should be between 1.0 and a minimum effective weight
	 * @param err
	 * @return
	 */
	public float computeSpeedWeight(float err) {
		float thresh = getVelocityErrorThreshold();
		float weight = (float) ((Math.abs(Math.abs(err)-thresh)) / Math.sqrt(err*err + thresh*thresh));
		if(weight > 1.0f)
			weight = 1.0f;
		else if(weight == 0)
			weight = 0.25f; // Lower bound
		
		return weight;
	}

	/**
	 * Computes the velocity error, which is left - right
	 * If left or right has not produced any data, than probably the robot is in the wrong position
	 * @return
	 */
	public float computeVelocityError() {
		float err = 0;
		int leftCount = data.getLeftCounts();
		int rightCount = data.getRightCounts();
		if(rightCount == 0 && leftCount == 0)
			return 0;
		if(rightCount == 0) // Produce biased conclusion for turning right
		{
			err = getVelocityErrorThreshold() + 1;
		}
		else if(leftCount == 0) // Produce biased conclusion for turning left
		{
			err = -getVelocityErrorThreshold()-1;
		}			
		else
		{
			err = velocityLeft - velocityRight;
		}
		return err;
	}

	/**
	 * Adjustment
	 * @param turnDirection
	 * @param weight
	 */
	public void AdjustRobotPosition(int turnDirection, float weight)
	{
		System.out.println("WEIGHT IS: " + weight);
		switch(turnDirection) // The ``visual blindness'' phase, as reported
		{
			case CMD_FLEFT:
				command(CMD_FLEFT, speedWeight, DEF_MOV_TIME-2,true,true);
				command(CMD_FORWARD, speedWeight, 1,true,false);
				command(CMD_BLEFT, speedWeight, DEF_MOV_TIME-2,true, false);
				setCorrectionPhase(true);
				command(CMD_FORWARD, 0.7f, 0, true,false);
				break;
			case CMD_FRIGHT:
				command(CMD_FRIGHT, speedWeight, DEF_MOV_TIME-2, true,true);
				command(CMD_FORWARD, speedWeight, 1,true,false);
				command(CMD_BRIGHT, speedWeight, DEF_MOV_TIME-2,true,false);
				setCorrectionPhase(true);
				command(CMD_FORWARD, 0.7f, 0, true,false);
				break;				
		}	
		
	}
	
	public void cancelCommand()
	{
		int cmd = assemble(CMD_CANCEL,0,0);
		transmit(cmd);
	}
	
	/**
	 * Sends a command to the robot via the relay
	 * @param direction
	 * @param speedWeight
	 * @param time
	 * @param force
	 * @param cancel_prev
	 */
	public void command(int direction, float speedWeight, int time, boolean force, boolean cancel_prev)
	{
		if(direction == CMD_FLEFT || direction == CMD_FRIGHT || direction == CMD_BLEFT || direction == CMD_BRIGHT)
			lastCommand = new int[] {direction, (int)(MAX_SPEED*speedWeight), time};
		if(LEVEL == ControlLevel.CONTROL_DEBUG || LEVEL == ControlLevel.EVALUATION)
			return;
		if(force)
		{
			if(cancel_prev)
			{
				try {
					Thread.sleep(300);
				} catch (InterruptedException exception) {
					exception.printStackTrace();
				}
				//System.out.println("Cancelling previous command...");
				cancelCommand();
			}
			int delay = (cancel_prev)?1:time+6;
			while(!isRobotReady(true,delay));
			System.out.println("Sending command to robot...");
			float weighted = (MAX_SPEED - MIN_SPEED)*speedWeight;
			int speed = 0;
			if (weighted + MIN_SPEED > MAX_SPEED)
				speed = MAX_SPEED;				
			else
				speed = (int)(MIN_SPEED + (MAX_SPEED - MIN_SPEED)*speedWeight);
			int cmd = assemble(direction,speed,time);
			transmit(cmd);
			
		}
		else 
		{
			if (isRobotReady(false,0))
			{
				// Response is returned fairly quickly, so may want to wait a bit before sending the next command
				int cmd = assemble(direction,(int)(MAX_SPEED*speedWeight),time);
				//System.out.println("Sending command to robot...");
				transmit(cmd);
			}
		}
	}

	/**
	 * Checks if the robot has ACKed 
	 * @param forced
	 * @param delay
	 * @return
	 */
	private boolean isRobotReady(boolean forced, int delay) {
		if(forced)
		{
			int resp = -1;
			//System.out.println("Entering loop");
			timer.setInitialDelay(delay*1000);
			timer.start();
			while(resp <= 0 && isThreadRunning())
			{			
					resp = waitForResponse();
			}
			timer.stop();
			//System.out.println("Exiting loop");
			return true; // Will always return true, because we're more interested in fact that the robot has actually sent something, meaning it's done
		}
		else
		{
			int resp = waitForResponse();
			return (resp==-1)?false:true; 
		}
	}
	
	ActionListener handleTimeout = new ActionListener()
	{
		public void actionPerformed(ActionEvent evt) 
		{
			System.out.println("Timeout");
			if(!isThreadRunning())
			{
				System.out.println("Thread is no longer running. Stopping timer...");
				timer.stop();
			}
			else
			{	
				if(!clientSocket.isClosed())
				{
					boolean success = transmit(assemble(CMD_CANCEL,0,0));//transmit(assemble(lastCommand[0],lastCommand[1],lastCommand[2]));
					if(success)
						timer.restart();
				}
							
			}
		}
	};

	private int waitForResponse()
	{
		int response = 0;
		try {
			if(fromClient.ready())
			{
				response = receive();
				//System.out.println("Received: " + response);
			}
				
			else 
				response = -1;
		} 
		catch (IOException exception)
		{					
			exception.printStackTrace();
		}
		return response;			
	}
	
	public void getData() {
		//this.velocityTiers = this.data.getVelocityTiers();
		//this.counters = this.data.getCountersPerTier();
		velocityLeft = filter.getVelocityLeftWeight()*data.getVelocityLeft();
		velocityRight = filter.getVelocityRightWeight()*data.getVelocityRight();
	}
	
	public synchronized void shutdown()
	{
		this.isRunning = false;
	}
	
	public boolean transmit(int message)
	{
		byte[] byte_data = convertToByteData(message);
		if((LEVEL == ControlLevel.NORMAL || LEVEL == ControlLevel.TRANSMIT_DEBUG))
		{
			try
			{
				toClient.write(byte_data);
				return true;
			
			}
			catch(IOException e)
			{
				System.out.println("Error transmitting message " + message);
				System.out.println("Trying to reconnect...");
				connect();
				return false;
			}
		}
		return true;
	}
	
	/**
	 * Receive response messages from robot
	 * @return
	 */
	public int receive() 
	{
		int recv = 0;
		int[] msg = new int[8];
		for (int pos=0;pos<msg.length;pos++)
		{
			try 
			{
				msg[pos] = fromClient.read();
				//System.out.println(msg[pos]);
			} 
			catch (IOException e) 
			{
				System.out.println("Error receiving message from robot");
				e.printStackTrace();
				return -1;
			}
		}
		recv = msg[0] + (msg[1] << 8) + (msg[2] << 16) + (msg[3] << 24);
		return recv;
		
	}
	
	public synchronized boolean isThreadRunning()
	{
		return this.isRunning;
	}
	
	public byte[] convertToByteData(int data) {
		byte[] bytes = new byte[4];
		ByteBuffer buffer = ByteBuffer.allocate(4);
		buffer.order(ByteOrder.LITTLE_ENDIAN);
		buffer.putInt(data);
		for(int i=0;i<buffer.limit();i++) {
			bytes[i] = buffer.get(i);
		}
		return bytes;
	}
	
	public int assemble(int cmd, int speed, int time)
	{
		if(speed > 1023)
			speed = 1023;
		if(time > 4)
			time = 4;
		int assembled = speed + (time << 13) + (cmd << 10);
		System.out.println("Assembled: " + assembled);
		return assembled;	
		
	}
	
	public boolean initResources()
	{
		try{
			serverSocket = new ServerSocket(PORT);
			System.out.println("### Server started ###");
			return true;
		} 
		catch (IOException e) 
		{
			System.out.println("Error establishing connections");
			e.printStackTrace();
			return false;
		}
	}
	
	public boolean connect()
	{
		try 
		{
			clientSocket = serverSocket.accept();
			System.out.println("Connected to Client");
			fromClient = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
			toClient = new DataOutputStream(clientSocket.getOutputStream());
			return true;
		} 
		catch (IOException e) 
		{
			System.out.println("Error establishing connections");
			e.printStackTrace();
			return false;
		}
	}	
	
	public synchronized void setVelocityErrorThreshold(float velocityErrorThreshold)
	{
		Controller.velocityErrorThreshold = velocityErrorThreshold;
		System.out.println("New Velocity Threshold Error:" + velocityErrorThreshold);
	}
	
	public synchronized float getVelocityErrorThreshold()
	{
		return velocityErrorThreshold;
	}
	
	public synchronized void setDefaultConclusionDepth(int depth)
	{
		CONCLUSION_DEPTH = depth;
		setLocalConclusionDepth(depth);
	}
	
	public void setLocalConclusionDepth(int depth) {
		this.localConclusionDepth = depth;		
	}

	public synchronized int getLocalConclusionDepth()
	{
		return localConclusionDepth;
	}
	
	public synchronized int getDefaultConclusionDepth()
	{
		return CONCLUSION_DEPTH;
	}
			
	public synchronized boolean isConclusionReady()
	{
		return isConclusionReady;
	}
	
	public synchronized void setIsConclusionReady(boolean isReady)
	{
		this.isConclusionReady = isReady;
	}
	
	public synchronized void setCurrentConclusion(int conclusion)
	{
		currentConclusion = conclusion;
	}
	
	public synchronized int getCurrentConclusion()
	{
		return currentConclusion;
	}
	
	public synchronized boolean isCorrectionPhase()
	{
		return correctionPhase;
	}
	
	public synchronized void setCorrectionPhase(boolean flag)
	{
		this.correctionPhase = flag;
	}
	
	public void reset()
	{
		this.currentConclusion = 0;
		this.error = 0;
		this.currentError = 0;
		numberConclusions = 0;
		this.idleCounter = 0;
		this.localConclusionDepth = CONCLUSION_DEPTH;
	}
}
