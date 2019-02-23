package org.usfirst.frc.team2485.util;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

public class ScalingMax extends WarlordsControlSystem {

	private double[] inVals, outVals;

	
	@Override
	public synchronized void setOutputs(PIDOutput... outputs) {
		super.setOutputs(outputs);
		outVals = new double[outputs.length]; 

	}
	
	@Override
	public synchronized void setSources(PIDSource... sources) {
		super.setSources(sources);
		inVals = new double[sources.length]; 
	}
	
	public double[] getInValues() {
		return inVals;
	}
	
	public double[] getOutValues() {
		return outVals;
	}
	
	@Override
	protected void calculate() {
		
		if (inVals.length != outVals.length) 
			throw new RuntimeException("invals and outvals are different lengths");
		
		for (int i = 0; i < inVals.length; i++) {
			inVals[i] = sources[i].pidGet();
		}
		
		double maxRatio = 1;
		for (double x : inVals) {
			double curRatio = Math.abs(x / setpoint);
			maxRatio = Math.max(maxRatio, curRatio);
		}
			
		for (int i = 0; i < outVals.length; i++) {
			outVals[i] = inVals[i] / maxRatio;
			outputs[i].pidWrite(outVals[i]);
		}

	}

}
