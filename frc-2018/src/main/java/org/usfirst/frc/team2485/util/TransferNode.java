package org.usfirst.frc.team2485.util;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class TransferNode implements PIDSource,  PIDOutput {
	
	double output;
	
	public TransferNode(double output) {
		this.output = output;
	}
	
	public double getOutput(){
		return output;
	}
	
	public void setOutput(double output){
		this.output = output;
	}
	
	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public double pidGet() {
		return output;
	}

	@Override
	public void setPIDSourceType(PIDSourceType arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void pidWrite(double output) {
		this.output = output;
		
	}

}
