package org.usfirst.frc.team2485.util;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * 
 * Used to invert the direction of a pot, as well as scale the units. 
 * 
 * @see ScaledPot
 * @author Anoushka Bose
 */

public class InvertedAbsoluteEncoder implements PIDSource {
	
	private AnalogPotentiometer pot;
		
	public InvertedAbsoluteEncoder(AnalogPotentiometer pot) {
		this.pot = pot;
	}
	
	@Override
	public double pidGet() {
		return (1 - pot.pidGet());
	}
	
	public double get() {
		return pidGet();
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return PIDSourceType.kDisplacement;
	}

}
