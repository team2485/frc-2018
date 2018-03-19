package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class WaitUntilArmUp extends Command {
	public WaitUntilArmUp() {
		requires(RobotMap.intake);
	}

	@Override
	protected boolean isFinished() {
		return Math.abs(RobotMap.arm.getThetaWrist() - RobotMap.wristEncoderWrapperDistance.pidGet()) < .02 && 
				Math.abs(RobotMap.arm.getThetaElbow() - RobotMap.arm.getElbowAngle()) < .03;
	}
	
	
	
}
