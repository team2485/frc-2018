package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class IntakeWithControllers extends InstantCommand {
	double pwm;
	
	public IntakeWithControllers(double pwm) {
		requires(RobotMap.intake);
		this.pwm = pwm;
	}
	
	@Override
	protected void initialize() {
		RobotMap.intake.setRollers(pwm);
	}

}
