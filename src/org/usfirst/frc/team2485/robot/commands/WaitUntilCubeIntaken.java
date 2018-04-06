package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.AutoLogger;
import org.usfirst.frc.team2485.util.Event.Type;

import edu.wpi.first.wpilibj.command.Command;

public class WaitUntilCubeIntaken extends Command {
	double maxCurrent = 30;
	double startTime;
	double startSpikeTime;
	boolean cubeIntaken = false;
	boolean overCurrent = false;
	long timeout;
	
	public WaitUntilCubeIntaken(long timeout) {
		requires(RobotMap.intake);
		this.timeout = timeout;
	}
	
	protected void initialize() {
		startTime = System.currentTimeMillis();
		AutoLogger.addEvent(Type.START, "WaitUntilCubeIntaken", "");
	} 
	
	protected void execute() {
		double averageCurrent = (RobotMap.intakeLeftTalon.getOutputCurrent() + RobotMap.intakeRightTalon.getOutputCurrent())/2;
		if (averageCurrent >= maxCurrent && !overCurrent) {
			startSpikeTime = System.currentTimeMillis();
			overCurrent = true;
		} else if (averageCurrent < maxCurrent) {
			overCurrent = false;
		}
		if (overCurrent && (System.currentTimeMillis() - startSpikeTime) > 500) {
			cubeIntaken = true;
		}
	}

	@Override
	protected boolean isFinished() {
		return cubeIntaken || System.currentTimeMillis() - startTime > timeout;
	}
	
	@Override
	protected void end() {
		// TODO Auto-generated method stub
		super.end();
		AutoLogger.addEvent(Type.STOP, "WaitUntilCubeIntaken", cubeIntaken ? "" : "timeout");
	}

}
