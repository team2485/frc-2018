package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.SetIntakeManual;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;

public class Eject extends CommandGroup {
	public Eject() {
		addSequential(new SetIntakeManual(-1));
		addSequential(new TimedCommand(1));
		addSequential(new SetIntakeManual(0));
	}
}
