package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import edu.wpi.first.wpilibj.command.Command;

public class ArmEmergencyControl extends Command{
	public ArmEmergencyControl() {
		requires(RobotMap.arm);
	}
	
	protected void execute() {
		WristWithControllers.isManual = true;
		RobotMap.arm.setElbowManual(ThresholdHandler.deadbandAndScale(OI.operator.getRawAxis(OI.XBOX_RYJOYSTICK_PORT), OI.XBOX_AXIS_DEADBAND, 0, 1));
		RobotMap.arm.setWristManual(ThresholdHandler.deadbandAndScale(OI.operator.getRawAxis(OI.XBOX_LYJOYSTICK_PORT), OI.XBOX_AXIS_DEADBAND, 0, 1));

	}
	
	@Override
	protected boolean isFinished() {
		return false;
	}

	
}
