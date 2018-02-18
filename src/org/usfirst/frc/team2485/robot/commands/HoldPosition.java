package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class HoldPosition extends InstantCommand {
	public HoldPosition() {
		requires(RobotMap.arm);
	}
	
	public void initialize() {
    	double theta2 = RobotMap.arm.getWristAngle();
    	if (RobotMap.arm.getElbowAngle() > 0) {
			RobotMap.arm.setThetaHigh(theta2);
		} else {
			RobotMap.arm.setThetaLow(theta2);
		}
		RobotMap.arm.setElbowPos(RobotMap.elbowEncoderWrapperDistance.pidGet());
	}
}
