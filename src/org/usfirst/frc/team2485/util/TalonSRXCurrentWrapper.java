package org.usfirst.frc.team2485.util;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class TalonSRXCurrentWrapper implements PIDSource {
	
	private TalonSRX canTalon;
	
	public TalonSRXCurrentWrapper(TalonSRX talonsrx) {
		this.canTalon = talonsrx;
		
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