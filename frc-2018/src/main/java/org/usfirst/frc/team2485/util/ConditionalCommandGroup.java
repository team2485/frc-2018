package org.usfirst.frc.team2485.util;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * @author Ben Dorsey
 */

public class ConditionalCommandGroup extends CommandGroup {
	private FinishedCondition finishedCondition;
	
	
	public ConditionalCommandGroup(FinishedCondition finishedCondition) {
		this.finishedCondition = finishedCondition;
	}

	@Override
	protected boolean isFinished() {
		return super.isFinished() || finishedCondition.isFinished();
	}
}
