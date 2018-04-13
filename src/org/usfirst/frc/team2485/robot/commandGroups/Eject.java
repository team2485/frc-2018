package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.SetIntakeManual;
import org.usfirst.frc.team2485.robot.commands.StartEjecting;
import org.usfirst.frc.team2485.robot.commands.WaitForArm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;

public class Eject extends CommandGroup {
	public Eject(boolean hard, boolean wait, boolean auto) {
		if (wait) {
			addSequential(new WaitForArm());
		}
		addSequential(new StartEjecting(hard, false));
		double time = 1;
		if (auto) {
			time = hard ? 0.3 : 0.5;
		}
		addSequential(new TimedCommand(time));
		addSequential(new SetIntakeManual(0));
	}
}
