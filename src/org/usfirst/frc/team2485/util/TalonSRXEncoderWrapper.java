package org.usfirst.frc.team2485.util;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class TalonSRXEncoderWrapper implements PIDSource{
	private TalonSRX talonsrx;
	private PIDSourceType pidSource;
	private double distancePerRevolution = 1;
	private double offset = 0;
	
	public TalonSRXEncoderWrapper(TalonSRX talonsrx, PIDSourceType pidSource) {
		this.talonsrx = talonsrx;
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
			return ((double)(talonsrx.getSensorCollection().getQuadraturePosition() + offset)/4096)*distancePerRevolution;
		} else {
			return ((double)(talonsrx.getSelectedSensorVelocity(0))/4096)*10*distancePerRevolution;
		}
	}
	
	public void setDistancePerRevolution(double distancePerRevolution) {
		this.distancePerRevolution = distancePerRevolution;
	}
	
	public void reset() {
		setPosition(0);
	}

	public void setPosition(double d) {
		this.offset = d * 4096 / distancePerRevolution - talonsrx.getSensorCollection().getQuadraturePosition();
		System.out.println(offset);
	}
	
	
}
