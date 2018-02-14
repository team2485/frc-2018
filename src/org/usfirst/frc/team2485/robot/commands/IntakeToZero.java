package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class IntakeToZero extends Command {
	
	double pwm;
	long duration;
	long startTime;
	
	public IntakeToZero(double pwm, long duration) {
//		requires(RobotMap.intake);
		this.pwm = pwm;
		this.duration = duration;
	}
	
	protected void initialize() {
		startTime = System.currentTimeMillis();
	}
	
	@Override
	protected void execute() {
		if(System.currentTimeMillis() - startTime < duration) {
			duration = System.currentTimeMillis();
			
			double percentFinished = (double)(System.currentTimeMillis() - startTime) / duration;
			double currentpwm = (double)(pwm)*(1-percentFinished);
			RobotMap.intake.setRollers(currentpwm);
		}
	}
	
	protected void end() {
		RobotMap.intake.setRollers(0);
	}

	@Override
	protected boolean isFinished() {
		return (System.currentTimeMillis() - startTime) > duration;
	}

}
