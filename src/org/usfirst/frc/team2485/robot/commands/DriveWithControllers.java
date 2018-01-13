package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveWithControllers extends Command {

    public DriveWithControllers() {
        requires(RobotMap.drivetrain);
        setInterruptible(true);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double y = -ThresholdHandler.deadbandAndScale(OI.XBOX.getRawAxis(OI.XBOX_LYJOYSTICK_PORT), RobotMap.drivetrain.THROTTLE_DEADBAND, 0, 1);
    	double x = ThresholdHandler.deadbandAndScale(OI.XBOX.getRawAxis(OI.XBOX_RXJOYSTICK_PORT), RobotMap.drivetrain.STEERING_DEADBAND, 0, 1);;
    	
    	RobotMap.drivetrain.simpleDrive(x, y);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}