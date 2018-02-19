package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class StopIntaking extends InstantCommand {
	public StopIntaking() {
	}
	@Override
	protected void initialize() {
		if (RobotMap.intake.getRollers() > 0) {
			RobotMap.intake.stopRollers();
		}
	}
}
