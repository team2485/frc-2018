package org.usfirst.frc.team2485.util;

public interface FinishedCondition {
	public boolean isFinished();
	public static final FinishedCondition FALSE_CONDITION = () -> {
		return false;
	};
}
