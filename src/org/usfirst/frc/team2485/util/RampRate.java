package org.usfirst.frc.team2485.util;

import edu.wpi.first.wpilibj.PIDOutput;

/**
 * Generic class to ramp an output (velocity, voltage, current, etc). Has unique up and down rates. 
 * @author Jeremy McCulloch
 */
public class RampRate extends WarlordsControlSystem {
	
	private double lastValue, upRampRate, downRampRate;
	
	public RampRate() {
		lastValue = 0;
	}
	
	public void setRampRates(double upRampRate, double downRampRate) {
		this.upRampRate = upRampRate;
		this.downRampRate = downRampRate;
	}
	
	/**
	 * Used to get next value by moving from the last value toward the desired value, without exceeding the ramp rate
	 * @param desired target value
	 * @return actual value
	 */
	
	public double getNextValue(double desired) {
		if ((lastValue > 0 && desired < 0) || (lastValue < 0 && desired > 0)) {
			desired = 0; // makes sure desired and lastValue have the same sign to make math easy
		}
		
		if (Math.abs(desired) > Math.abs(lastValue)) {
			if (Math.abs(desired - lastValue) > upRampRate) {
				if (desired > 0) {
					lastValue += upRampRate;
				} else {
					lastValue -= upRampRate;
				}
			} else {
				lastValue = desired;
			}
		} else {
			if (Math.abs(desired - lastValue) > downRampRate) {
				if (lastValue > 0) {
					lastValue -= downRampRate;

				} else {
					lastValue += downRampRate;

				}
			} else {
				lastValue = desired;
			}
		}
		
		return lastValue;
	}
	
	/**
	 * Used to immediately set the last value, potentially not following ramp rate
	 * @param lastValue new value to be treated as the last value
	 */
	public void setLastValue(double lastValue) {
		this.lastValue = lastValue;
	}
	

	@Override
	protected void calculate() {
		double val = getNextValue(setpoint);
		for (PIDOutput out : outputs) {
			out.pidWrite(val);
		}
	}
	
	@Override
	public void disable() {
		setLastValue(0);
		setpoint = 0;
		super.disable();
	}
}
