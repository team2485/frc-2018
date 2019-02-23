package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.AutoLogger;
import org.usfirst.frc.team2485.util.Event.Type;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class ZeroEncoders extends InstantCommand{
	public ZeroEncoders() {
		requires(RobotMap.driveTrain);
	}
	
	@Override
	protected void initialize() {
		RobotMap.driveTrain.zeroEncoders();
		AutoLogger.addEvent(Type.START, "ZeroEncoders", ""); // instant
	}
}
