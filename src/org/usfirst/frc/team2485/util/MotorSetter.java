package org.usfirst.frc.team2485.util;

import edu.wpi.first.wpilibj.PIDOutput;

public class MotorSetter extends WarlordsControlSystem{

	
	
	
	@Override
	protected void calculate() {
		for (PIDOutput out : super.outputs) {
			out.pidWrite(setpoint);
		}
	}
	
	@Override
	public void disable() {
		super.disable();
		for (PIDOutput out : super.outputs) {
			out.pidWrite(0);
		}
	}

}
