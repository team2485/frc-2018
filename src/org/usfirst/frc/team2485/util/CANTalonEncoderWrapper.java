package org.usfirst.frc.team2485.util;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class CANTalonEncoderWrapper implements PIDSource{
	private CANTalon cantalon;
	private PIDSourceType pidSource;
	private double distancePerRevolution = 1;
	
	public CANTalonEncoderWrapper(CANTalon cantalon, PIDSourceType pidSource) {
		this.cantalon = cantalon;
		this.pidSource = pidSource;
	}
	
	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		this.pidSource = pidSource;
	}
	
	@Override
	public PIDSourceType getPIDSourceType() {
		return pidSource;
	}
	
	@Override
	public double pidGet() {
		if(pidSource == PIDSourceType.kDisplacement) {
			return ((double)(cantalon.getEncPosition())/4096)*distancePerRevolution;
		} else {
			return ((double)(cantalon.getEncVelocity())/4096)*10*distancePerRevolution;
		}
	}
	
	public void setDistancePerRevolution(double disancePerRevolution) {
		this.distancePerRevolution = distancePerRevolution;
	}
	
	
}
