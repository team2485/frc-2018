package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.AutoLogger;
import org.usfirst.frc.team2485.util.Event.Type;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class ResetDriveTrain extends InstantCommand{
	public ResetDriveTrain() {
		requires(RobotMap.driveTrain);
	}
	
	@Override
	protected void initialize() {
		RobotMap.driveTrain.reset();
		System.out.println("ResetDriveTrain");
		AutoLogger.addEvent(Type.START, "ResetDriveTrain", ""); // instant
	}

}
