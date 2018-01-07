package org.usfirst.frc.team2485.util;

import edu.wpi.first.wpilibj.command.Command;

public class CommandTimeout extends Command {

	private Command innerCommand;
	private long startTime;
	private int timeout;

	public CommandTimeout(int timeout) {
		this(null, timeout);
	}
	
	public CommandTimeout(Command innerCommand, int timeout) {
		this.innerCommand = innerCommand;
		this.timeout = timeout;
	}

	@Override
	protected void initialize() {
		System.out.println("Started timedCommand");
		if (innerCommand != null) {
			innerCommand.start();
		}
		startTime = System.currentTimeMillis();
	}

	@Override
	protected void execute() {
		if ((System.currentTimeMillis()-startTime) > timeout && innerCommand != null) {
			System.out.println("Timed out");
			innerCommand.cancel();
		}
	}

	@Override
	public boolean isFinished() {
		if (innerCommand == null) {
			return isTimedOut();
		} else {
			System.out.println(innerCommand.isRunning());
			if (innerCommand.timeSinceInitialized() > 0.050) {
				return !innerCommand.isRunning() || innerCommand.isCanceled();
			} else {
				return false;
			}
		}
	}

	@Override
	protected void end() {
		if (innerCommand != null) {
			innerCommand.cancel();
			System.out.println("end");
		}
	}

	@Override
	protected void interrupted() {
		end();
	}
	
	@Override
	public String toString() {
		return super.toString() + " -> " + innerCommand;
	}
}