package org.usfirst.frc.team2485.util;

import edu.wpi.first.wpilibj.PIDOutput;

public class MotorSetter extends WarlordsControlSystem{

	
	
	
	@Override
	protected synchronized void calculate() {
		// TODO Auto-generated method stub
		for (PIDOutput out : super.outputs) {
			out.pidWrite(setpoint);
		}
	}
	
	@Override
	public synchronized void disable() {
		super.disable();
		for (PIDOutput out : super.outputs) {
			out.pidWrite(0);
		}
	}

}
