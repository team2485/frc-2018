package org.usfirst.frc.team2485.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

/**
 * @author Nicholas Contreras
 */

public class JoystickAxisButton extends Button {

	private Joystick stick;
	private int axis;
	private double minValue, maxValue;

	public JoystickAxisButton(Joystick stick, int axis, double minValue, double maxValue) {

		this.stick = stick;

		this.axis = axis;

		this.minValue = minValue;
		this.maxValue = maxValue;
	}

	@Override
	public boolean get() {
		double val = stick.getRawAxis(axis);

		return val >= minValue && val <= maxValue;
	}
}
