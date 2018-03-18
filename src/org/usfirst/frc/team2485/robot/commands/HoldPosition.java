package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class HoldPosition extends InstantCommand {
	public HoldPosition() {
		requires(RobotMap.arm);
	}
	
	public void initialize() {
    	RobotMap.arm.setThetaWrist(RobotMap.wristEncoderWrapperDistance.pidGet());
		RobotMap.arm.setThetaElbow(RobotMap.elbowEncoderWrapperDistance.pidGet());
	}
}
