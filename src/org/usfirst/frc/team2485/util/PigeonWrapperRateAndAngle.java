package org.usfirst.frc.team2485.util;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PigeonWrapperRateAndAngle implements PIDSource {
	public static enum Units {
		DEGS, RADS
	}
	private PIDSourceType pidSource;
	private Units units;
	public PigeonWrapperRateAndAngle(PIDSourceType pidSource, Units units) {
		this.pidSource = pidSource;
		this.units = units;
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
			return (units == Units.RADS) ? Math.PI / 180 * RobotMap.pigeon.getFusedHeading() : RobotMap.pigeon.getFusedHeading();
			
		} else {
			double[] ypr = new double[3];
			RobotMap.pigeon.getRawGyro(ypr);
			
			return (units == Units.RADS) ? Math.PI / 180 * ypr[0] : ypr[0];
		}
	}
	
	
}
