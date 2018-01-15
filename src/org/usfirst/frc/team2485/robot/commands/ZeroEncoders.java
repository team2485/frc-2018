package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class ZeroEncoders extends InstantCommand{
	public ZeroEncoders() {
		requires(RobotMap.drivetrain);
	}
	
	@Override
	protected void initialize() {
		RobotMap.drivetrain.zeroEncoders();
	}
}
