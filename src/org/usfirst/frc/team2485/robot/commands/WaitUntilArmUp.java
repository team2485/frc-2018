package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.command.Command;

public class WaitUntilArmUp extends Command{
	public WaitUntilArmUp() {
		requires(RobotMap.intake);
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		System.out.println("Wrist Error " + (RobotMap.arm.wristAngPID.getSetpoint() - RobotMap.arm.getWristAngle()));
		System.out.println("Elbow Error " + (RobotMap.arm.elbowAngPID.getSetpoint() - RobotMap.arm.getElbowAngle()));
		return Math.abs(RobotMap.arm.wristAngPID.getSetpoint() - RobotMap.arm.getWristAngle()) < .02 && Math.abs(RobotMap.arm.elbowAngPID.getSetpoint() - RobotMap.arm.getElbowAngle()) < .03;
	}
	
	
	
}
