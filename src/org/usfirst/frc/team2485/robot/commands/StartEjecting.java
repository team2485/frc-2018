package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.AutoLogger;
import org.usfirst.frc.team2485.util.Event.Type;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class StartEjecting extends InstantCommand {
	
	private boolean hard;
	public StartEjecting(boolean hard) {
		this.hard = hard;
		
	}
	@Override
	protected void initialize() {
		if (hard) {
			if (RobotMap.arm.getElbowAngle() > 0) {
				RobotMap.intake.setRollers(-0.6);
			} else {
				RobotMap.intake.setRollers(-0.73);
			}
		} else {
			if (RobotMap.arm.getElbowAngle() > 0) {
				RobotMap.intake.setRollers(-0.25);
			} else {
				RobotMap.intake.setRollers(-0.4);
			}
		}
		AutoLogger.addEvent(Type.START, "StartEjecting", "");
	}
}
