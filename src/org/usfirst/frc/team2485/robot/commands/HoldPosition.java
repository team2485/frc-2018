package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class HoldPosition extends InstantCommand {
	public HoldPosition() {
		requires(RobotMap.arm);
	}
	
	public void initialize() {
		if (!OI.operator.getRawButton(OI.XBOX_LBUMPER_PORT) && !OI.operator.getRawButton(OI.XBOX_RBUMPER_PORT) && !OI.operator.getRawButton(OI.XBOX_A_PORT) && !OI.operator.getRawButton(OI.XBOX_B_PORT) && !OI.operator.getRawButton(OI.XBOX_X_PORT) && !OI.operator.getRawButton(OI.XBOX_Y_PORT) && !OI.operator.getRawButton(OI.XBOX_BACK_BUTTON) && OI.operator.getRawButton(OI.XBOX_START_BUTTON)
				&& !OI.operatorBackup.getRawButton(OI.XBOX_LBUMPER_PORT) && !OI.operatorBackup.getRawButton(OI.XBOX_RBUMPER_PORT) && !OI.operatorBackup.getRawButton(OI.XBOX_A_PORT) && !OI.operatorBackup.getRawButton(OI.XBOX_B_PORT) && !OI.operatorBackup.getRawButton(OI.XBOX_X_PORT) && !OI.operatorBackup.getRawButton(OI.XBOX_Y_PORT) && !OI.operatorBackup.getRawButton(OI.XBOX_BACK_BUTTON) && !OI.operatorBackup.getRawButton(OI.XBOX_START_BUTTON)) {
			RobotMap.arm.setThetaWrist(RobotMap.wristEncoderWrapperDistance.pidGet());
			RobotMap.arm.setThetaElbow(RobotMap.elbowEncoderWrapperDistance.pidGet());
		}
	}
}
