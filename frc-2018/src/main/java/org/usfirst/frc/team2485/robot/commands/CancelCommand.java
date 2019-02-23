package org.usfirst.frc.team2485.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class CancelCommand extends InstantCommand {
	private Command c;
	public CancelCommand(Command c) {
		this.c = c;
	}
	@Override
	protected void initialize() {
		c.cancel();
	}
}
