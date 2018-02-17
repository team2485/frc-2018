package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import edu.wpi.first.wpilibj.command.Command;


public class IntakeWithControllers extends Command {
	
	double pwm;
	boolean byJoysticks;
	
	public IntakeWithControllers() {
		requires(RobotMap.intake);
		setInterruptible(true);
		byJoysticks = true;
		pwm = 0;
	}
	
	public IntakeWithControllers(double pwm) {
		this.pwm = pwm;
		requires(RobotMap.intake);
		setInterruptible(true);
		byJoysticks = false;
	}
	
	@Override
	protected void execute() {
		if (byJoysticks) {
			pwm = ThresholdHandler.deadbandAndScale(OI.operator.getRawAxis(OI.XBOX_RYJOYSTICK_PORT), .2, 0, 1);
		}
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
