package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.ArmSetSetpoint;
import org.usfirst.frc.team2485.robot.subsystems.Arm.ArmSetpoint;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ArmToScale extends CommandGroup {
	public ArmToScale(ArmSetpoint setpoint) {
		addSequential(new ArmSetSetpoint(setpoint));
	}
}
