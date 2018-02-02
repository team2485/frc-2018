package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class IntakeWithControllers extends Command {
	double pwm;
	
	public IntakeWithControllers(double pwm) {
//		requires(RobotMap.intake);
		this.pwm = pwm;
	}
	
	@Override
	protected void initialize() {
		RobotMap.intake.setRollers(pwm);
	}
	
	public void stop() {
		RobotMap.intake.stopRollers();
	}
	
	@Override
	protected boolean isFinished() {
		return false;
	}

}
