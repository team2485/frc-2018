package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class ZeroArmEncoders extends InstantCommand{
	public ZeroArmEncoders() {
		requires(RobotMap.arm);
	}
	
	@Override
	protected void initialize() {
		RobotMap.arm.zeroElbowEnc();
		RobotMap.arm.zeroWristEnc();
	}
}
