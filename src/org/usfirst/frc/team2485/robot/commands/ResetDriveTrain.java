package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class ResetDriveTrain extends InstantCommand{
	public ResetDriveTrain() {
		requires(RobotMap.drivetrain);
	}
	
	@Override
	protected void initialize() {
		RobotMap.drivetrain.reset();
	}

}