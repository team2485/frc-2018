package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class IntakeWithControllers extends Command {
	public IntakeWithControllers() {
		requires(RobotMap.intake);
	}
	
	//Runs while the button is pressed (wasn't sure which one)
	protected void execute() {
		//xbox goes here
		RobotMap.intake.startRollers();
	}
	
	@Override
	protected boolean isFinished() {
		return false;
	}

}
