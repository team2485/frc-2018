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
			RobotMap.pigeon.getYawPitchRoll(ypr);
			double delta = ypr[0]; 
			if(delta<-180) {
				delta+=360;
			}
			else if(delta>180) {
				delta-=360;
			}
			double rate = delta*60;
			
			return (units == Units.RADS) ? Math.PI / 180 * rate : rate;
		}
	}
	
	
}
