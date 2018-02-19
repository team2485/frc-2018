package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.util.FinishedCondition;

import edu.wpi.first.wpilibj.command.Command;

public class Wait extends Command {
	private FinishedCondition fc;
	public Wait(FinishedCondition fc) {
		this.fc = fc;
	}
	@Override
	protected boolean isFinished() {
		return fc.isFinished();
	}
	

}
