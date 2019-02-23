package org.usfirst.frc.team2485.util;

import edu.wpi.first.wpilibj.PIDOutput;

/**
 * @author Ben Dorsey
 */

public class PIDOutputWrapper implements PIDOutput {
	private PIDOutput pidOutput;
	public void setPidOutput(PIDOutput pidOutput) {
		this.pidOutput = pidOutput;
	}
	@Override
	public void pidWrite(double output) {
		pidOutput.pidWrite(output);
	}
	
}
