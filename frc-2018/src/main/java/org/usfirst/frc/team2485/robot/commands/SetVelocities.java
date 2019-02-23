package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class SetVelocities extends Command{
private double linearVelocity, angularVelocity;
	
	public SetVelocities(double linearVelocity, double angularVelocity) {
		requires(RobotMap.driveTrain);
		this.linearVelocity = linearVelocity;
		this.angularVelocity = angularVelocity;
	}
	
	@Override
	protected void execute() {
		RobotMap.driveTrain.setVelocities(linearVelocity, angularVelocity);;
	}

	@Override
	protected boolean isFinished() {
		return false;
	}
	

}
