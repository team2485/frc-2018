package org.usfirst.frc.team2485.util;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class CANTalonCurrentWrapper implements PIDSource {
	
	private CANTalon canTalon;
	
	public CANTalonCurrentWrapper(CANTalon cantalon) {
		this.canTalon = cantalon;
		
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public double pidGet() {
  		return canTalon.getOutputCurrent();
	}

}