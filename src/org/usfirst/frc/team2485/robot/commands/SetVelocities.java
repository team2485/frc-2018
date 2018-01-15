package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class SetVelocities extends InstantCommand{
private double linearVelocity, angularVelocity;
	
	public SetVelocities(double linearVelocity, double angularVelocity) {
		this.linearVelocity = linearVelocity;
		this.angularVelocity = angularVelocity;
	}
	
	@Override
	protected void initialize() {
		RobotMap.drivetrain.setVelocities(linearVelocity, angularVelocity);;
	}
	

}
