package org.usfirst.frc.team2485.util;

import edu.wpi.first.wpilibj.PIDOutput;

public class WarlordsDistanceController extends WarlordsControlSystem {

	private double error, kGain, kOffset;
	
	private double sensorVal;
	
	private double minOutput, maxOutput;
	private double minInput, maxInput;
	
	private boolean continuous;
	
	public void setConstants(double kGain, double kOffset) {
		this.kGain = kGain;
		this.kOffset = kOffset;
	}
	
	public void setOutputRange(double minOutput, double maxOutput) {
		this.minOutput = minOutput;
		this.maxOutput = maxOutput;
	}
	
	public void setInputRange(double minInput, double maxInput) {
		this.minInput = minInput;
		this.maxInput = maxInput;
	}
	
	public void setContinuous(boolean continuous) {
		this.continuous = continuous;
	}
	
	public boolean isContinuous() {
		return continuous;
	}
	
	public double getError() {
		return setpoint - sources[0].pidGet();
	}
	
	
	protected void calculate() {
		
		sensorVal = sources[0].pidGet();
		error = setpoint - sensorVal;
		
		while (continuous && Math.abs(error) > (maxInput - minInput) / 2) {
		
			if (error > 0) {
				error -= maxInput - minInput;
			} else {
				error += maxInput - minInput;
			}
			
		}
		
		double output = - kOffset + Math.sqrt((kGain * Math.abs(error)) + (Math.pow(kOffset, 2))); 
		
		if (error < 0) {
			output *= -1;
		}
		
		if (output > maxOutput) {
			output = maxOutput;
		} else if (output < minOutput) {
			output = minOutput;
		}
		
		for (PIDOutput PIDout : super.outputs) {
			PIDout.pidWrite(output);
		}
		
	}

	
}
