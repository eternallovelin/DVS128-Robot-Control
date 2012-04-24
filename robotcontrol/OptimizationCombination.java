package jaer.myjaer.robotcontrol;

/**
 * This class is used for the brute optimization routine. It stores the combination of values for all 8 key variables along with the 
 * final ratio (correct/incorrect conclusion) score
 * @author vanxa
 *
 */

public class OptimizationCombination {
	private int completeOnConfirmedSignals, conclusionDepth, confirmThreshold, neuronsOnRadius, signalArrivalThreshold;
	private float expectedInitialSignalVelocity, velocityErrorThreshold, score;
	
	public OptimizationCombination(int complete, int conclusions, int confirmThresh, int neurons, int signalArr, 
			float expInitVel, float velErrThresh, float score)
	{
		this.completeOnConfirmedSignals = complete;
		this.conclusionDepth = conclusions;
		this.confirmThreshold = confirmThresh;
		this.neuronsOnRadius = neurons;
		this.signalArrivalThreshold = signalArr;
		this.expectedInitialSignalVelocity = expInitVel;
		this.velocityErrorThreshold = velErrThresh;
		this.score = score;
	}

	/**
	 * @return the completeOnConfirmedSignals
	 */
	public int getCompleteOnConfirmedSignals() {
		return this.completeOnConfirmedSignals;
	}

	/**
	 * @param completeOnConfirmedSignals the completeOnConfirmedSignals to set
	 */
	public void setCompleteOnConfirmedSignals(int completeOnConfirmedSignals) {
		this.completeOnConfirmedSignals = completeOnConfirmedSignals;
	}

	/**
	 * @return the conclusionDepth
	 */
	public int getConclusionDepth() {
		return this.conclusionDepth;
	}

	/**
	 * @param conclusionDepth the conclusionDepth to set
	 */
	public void setConclusionDepth(int conclusionDepth) {
		this.conclusionDepth = conclusionDepth;
	}

	/**
	 * @return the confirmThreshold
	 */
	public int getConfirmThreshold() {
		return this.confirmThreshold;
	}

	/**
	 * @param confirmThreshold the confirmThreshold to set
	 */
	public void setConfirmThreshold(int confirmThreshold) {
		this.confirmThreshold = confirmThreshold;
	}

	/**
	 * @return the neuronsOnRadius
	 */
	public int getNeuronsOnRadius() {
		return this.neuronsOnRadius;
	}

	/**
	 * @param neuronsOnRadius the neuronsOnRadius to set
	 */
	public void setNeuronsOnRadius(int neuronsOnRadius) {
		this.neuronsOnRadius = neuronsOnRadius;
	}

	/**
	 * @return the signalArrivalThreshold
	 */
	public int getSignalArrivalThreshold() {
		return this.signalArrivalThreshold;
	}

	/**
	 * @param signalArrivalThreshold the signalArrivalThreshold to set
	 */
	public void setSignalArrivalThreshold(int signalArrivalThreshold) {
		this.signalArrivalThreshold = signalArrivalThreshold;
	}

	/**
	 * @return the expectedInitialSignalVelocity
	 */
	public float getExpectedInitialSignalVelocity() {
		return this.expectedInitialSignalVelocity;
	}

	/**
	 * @param expectedInitialSignalVelocity the expectedInitialSignalVelocity to set
	 */
	public void setExpectedInitialSignalVelocity(float expectedInitialSignalVelocity) {
		this.expectedInitialSignalVelocity = expectedInitialSignalVelocity;
	}

	/**
	 * @return the velocityErrorThreshold
	 */
	public float getVelocityErrorThreshold() {
		return this.velocityErrorThreshold;
	}

	/**
	 * @param velocityErrorThreshold the velocityErrorThreshold to set
	 */
	public void setVelocityErrorThreshold(float velocityErrorThreshold) {
		this.velocityErrorThreshold = velocityErrorThreshold;
	}

	/**
	 * @return the score
	 */
	public float getScore() {
		return this.score;
	}

	/**
	 * @param score the score to set
	 */
	public void setScore(float score) {
		this.score = score;
	}
	
	
}
