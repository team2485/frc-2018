package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;


public class SetIntakeManual extends Command {
	
	double pwm;
	
	public SetIntakeManual(double pwm) {
		this.pwm = pwm;
		requires(RobotMap.intake);
		setInterruptible(true);
	}
	
	@Override
	protected void initialize() {
		RobotMap.intake.setRollers(pwm);
	}

	@Override
	protected boolean isFinished() {
		return false;
	}
	

}
