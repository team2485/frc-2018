package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class ArmEmergencyControl extends Command{
	public ArmEmergencyControl() {
		requires(RobotMap.arm);
	}
	
	protected void execute() {
		RobotMap.arm.setElbowManual(OI.operator.getRawAxis(OI.XBOX_RYJOYSTICK_PORT));
		RobotMap.arm.setWristManual(OI.operator.getRawAxis(OI.XBOX_LYJOYSTICK_PORT));

	}
	
	@Override
	protected boolean isFinished() {
		return false;
	}

	
}
