package org.usfirst.frc.team2485.util;

/**
 * 
 * Utililty class for handling inputs from controllers. 
 * 
 * @author Patrick Wamsley
 * @author Anoushka Bose
 * @author Aidan Fay
 * @author Jeremy McCulloch
 */
public class ThresholdHandler { 
	
	public static final double STANDARD_THRESHOLD = 0.1; 

	/**
	 * Thresholds and scales values linearly.                                                                      
	 * <code>((max - min) / (1 - t)) * (v - t) + min;</code> where <code>(max - min) / (1 - t)</code> is the slope.
	 * Derived from fact that an input value of the threshold should return min : ( <code>f(threshold) = min</code> )
	 * and an input value of 1 should return max : ( <code> f(1) = max </code> ) 
	 *  
	 * @param val = raw input from controllers/joysticks
	 * @param threshold = deadband threshold  (values less than this are ignored) 
	 * @param absoluteMin = absolute value of min range (must be greater than 0)
	 * @param absoluteMax = absolute value of max range (must be less than 1) 
	 * 
	 * @return Corrected input from joystick. If input is below threshold, 0 is returned. 
	 * 		   If not, input is scaled between (min, max). 
	 * 
	 */
	public static double deadbandAndScale(double val, double threshold, double absoluteMin, double absoluteMax) {
		
		if (Math.abs(val) <= threshold)
			return 0; 
		
		double returnVal = ((absoluteMax - absoluteMin) / (1 - threshold)) * (Math.abs(val) - threshold) + absoluteMin; 
		
		return val > 0 ? returnVal : -returnVal; 
	}
	
	/**
	 * Thresholds and scales linearly using linear ramps with potentially different slopes
	 * @param val = raw input from controllers/joysticks
	 * @param threshold = deadband threshold  (values less than this are ignored) 
	 * @param absoluteMin = absolute value of min range (must be greater than 0)
	 * @param absoluteMidOut = absolute value of what absoluteMidIn should map to
	 * @param absoluteMidIn = ramps using one slope when less that absoluteMidIn, and another when greater
	 * @param absoluteMax = absolute value of max range (must be less than 1)
	 *  
	 * @return Corrected input from joystick. If input is below threshold, 0 is returned. 
	 * 		   If not, input is scaled between (min, max) with absoluteMidIn mapping to absoluteMidOut	 */
	public static double deadbandAndScaleDualRamp(double val, double threshold, double absoluteMin, double absoluteMidIn, double absoluteMidOut, 
			 double absoluteMax) {
		
		double returnVal;
		
		if (Math.abs(val) <= threshold) {
			return 0; 
		} else if (Math.abs(val) <= absoluteMidIn) {
			returnVal = ((absoluteMidOut - absoluteMin) / (absoluteMidIn - threshold)) * (Math.abs(val) - threshold) + absoluteMin; 
		} else {
			returnVal = ((absoluteMax - absoluteMidOut) / (1 - absoluteMidIn)) * (Math.abs(val) - absoluteMidIn) + absoluteMidOut; 
		}
		
		return val > 0 ? returnVal : -returnVal; 

	}
	
	
	
}
