package org.usfirst.frc.team2485.util;


import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PigeonWrapperRateAndAngle implements PIDSource {
	public static enum Units {
		DEGS, RADS
	}
	private PIDSourceType pidSource;
	private Units units;
	
	PigeonIMU gyro;
	
	public PigeonWrapperRateAndAngle(PigeonIMU gyro, PIDSourceType pidSource, Units units) {
		this.gyro = gyro;
		this.pidSource = pidSource;
		this.units = units;
	}
	
	public void reset() {
		gyro.setFusedHeading(0, 50); 
		gyro.setYaw(0, 50); 
	}
	
	public PigeonIMU getPigeon() {
		return gyro;
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
		if (pidSource == PIDSourceType.kDisplacement) {
			return (units == Units.RADS) ? Math.PI / 180 * -1 * gyro.getFusedHeading() : -1 * gyro.getFusedHeading();
			
		} else {
			double[] xyz = new double[3];
			gyro.getRawGyro(xyz);
			
			return (units == Units.RADS) ? Math.PI / 180 * -1 * xyz[2] : -1 * xyz[2];
		}
	}
	
	
}
