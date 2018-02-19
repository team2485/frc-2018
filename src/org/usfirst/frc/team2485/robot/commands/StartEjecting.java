package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class StartEjecting extends InstantCommand {
	@Override
	protected void initialize() {
		if (RobotMap.arm.getElbowAngle() > 0) {
			RobotMap.intake.setRollers(-0.25);
		} else {
			RobotMap.intake.setRollers(-1);
		}
	}
}
