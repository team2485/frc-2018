package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.ArmSetSetpoint;
import org.usfirst.frc.team2485.robot.commands.Wait;
import org.usfirst.frc.team2485.robot.subsystems.Arm;
import org.usfirst.frc.team2485.robot.subsystems.Arm.ArmSetpoint;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ArmToScale extends CommandGroup {
	public ArmToScale(ArmSetpoint setpoint) {
		addSequential(new ArmSetSetpoint(ArmSetpoint.SWITCH));
		addSequential(new Wait(() -> {
			return RobotMap.arm.getWristAngle() > Arm.MIN_WRIST_LIFTING_POSITION;
		}));
		addSequential(new ArmSetSetpoint(setpoint));
	}
}
