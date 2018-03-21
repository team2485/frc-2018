package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class WaitUntilCubeIntaken extends Command {
	double maxCurrent = 20;
	double startTime;
	double timeout = 10000;
	
	public WaitUntilCubeIntaken() {
		// TODO Auto-generated constructor stub
		requires(RobotMap.intake);
	}
	
	protected void initialize() {
		startTime = System.currentTimeMillis();
	} 

	@Override
	protected boolean isFinished() {
		double averageCurrent = (RobotMap.intakeLeftTalon.getOutputCurrent() + RobotMap.intakeRightTalon.getOutputCurrent())/2;
		return averageCurrent >= maxCurrent || System.currentTimeMillis() - startTime > timeout;
	}

}
