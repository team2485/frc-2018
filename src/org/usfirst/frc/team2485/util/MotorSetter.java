package org.usfirst.frc.team2485.util;

import edu.wpi.first.wpilibj.PIDOutput;

public class MotorSetter extends WarlordsControlSystem{

	
	
	
	@Override
	protected void calculate() {
		// TODO Auto-generated method stub
		for (PIDOutput out : super.outputs) {
			out.pidWrite(setpoint);
		}
	}

}
