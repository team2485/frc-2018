package org.usfirst.frc.team2485.util;

import edu.wpi.first.wpilibj.PIDOutput;

public class LowPassFilter extends WarlordsControlSystem {
	
	private double filterCoefficient;
	private double lastValue;
	
	public void setFilterCoefficient(double filterCoefficient) {
		this.filterCoefficient = filterCoefficient;
	}
	
	@Override
	public synchronized void disable() {
		lastValue = 0;
		super.disable();
	}
	
	@Override
	protected void calculate() {
		lastValue += filterCoefficient * (setpoint - lastValue);
		for (PIDOutput output : outputs) {
			output.pidWrite(lastValue);
		}
	}

}
