package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class SetVelocities extends Command{
private double linearVelocity, angularVelocity;
	
	public SetVelocities(double linearVelocity, double angularVelocity) {
		this.linearVelocity = linearVelocity;
		this.angularVelocity = angularVelocity;
	}
	
	@Override
	protected void execute() {
		RobotMap.drivetrain.setVelocities(linearVelocity, angularVelocity);;
	}

	@Override
	protected boolean isFinished() {
		return false;
	}
	

}
