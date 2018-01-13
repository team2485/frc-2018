package org.usfirst.frc.team2485.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * @author Ben Dorsey
 */

public class TalonSRXWrapper implements SpeedController, PIDOutput {
	private TalonSRX talon;
	private ControlMode controlMode;
	private boolean isInverted = false;
	
	public TalonSRXWrapper(ControlMode controlMode, TalonSRX talon) {
		this.talon = talon;
		this.controlMode = controlMode;
		
	}

	@Override
	public void pidWrite(double output) {
		set(output);
	}

	@Override
	public void set(double speed) {
		if (isInverted) {
			speed *= -1;
		}
		talon.set(controlMode, speed);
	}

	@Override
	public double get() {
		return talon.getMotorOutputPercent();
	}

	@Override
	public void setInverted(boolean isInverted) {
		this.isInverted = isInverted;
	}

	@Override
	public boolean getInverted() {
		return isInverted;
	}

	@Override
	public void disable() {
	}

	@Override
	public void stopMotor() {
	}

}
