package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class InitializeArmEncoders extends InstantCommand{
	public InitializeArmEncoders() {
		requires(RobotMap.arm);
	}
	
	@Override
	protected void initialize() {
		RobotMap.arm.initElbowEnc();
		RobotMap.arm.initWristEnc();
	}
}
