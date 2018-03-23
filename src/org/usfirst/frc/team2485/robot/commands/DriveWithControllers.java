package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.Robot;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveWithControllers extends Command {

    public DriveWithControllers() {
        requires(RobotMap.driveTrain);
        setInterruptible(true);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double throttle = ThresholdHandler.deadbandAndScale(OI.driver.getRawAxis(OI.XBOX_RTRIGGER_PORT), OI.XBOX_TRIGGER_DEADBAND, 0, 1) 
    			- ThresholdHandler.deadbandAndScale(OI.driver.getRawAxis(OI.XBOX_LTRIGGER_PORT), OI.XBOX_TRIGGER_DEADBAND, 0, 1);
    	
    	double steering = ThresholdHandler.deadbandAndScale(OI.driver.getRawAxis(OI.XBOX_LXJOSYSTICK_PORT), 
    			OI.XBOX_AXIS_DEADBAND, 0, 1);
    	
    	
    	
       	boolean quickturn = OI.driver.getRawButton(OI.XBOX_X_PORT);
       	
       	if (!quickturn && steering == 0 && throttle == 0) {
       		throttle = ThresholdHandler.deadbandAndScale(OI.driverBackup.getRawAxis(OI.XBOX_RTRIGGER_PORT), OI.XBOX_TRIGGER_DEADBAND, 0, 1) 
        			- ThresholdHandler.deadbandAndScale(OI.driverBackup.getRawAxis(OI.XBOX_LTRIGGER_PORT), OI.XBOX_TRIGGER_DEADBAND, 0, 1);
        	steering = ThresholdHandler.deadbandAndScale(OI.driverBackup.getRawAxis(OI.XBOX_LXJOSYSTICK_PORT), 
        			0.25, 0, 1);
           	quickturn = OI.driverBackup.getRawButton(OI.XBOX_X_PORT);
       	}
       	
       	
    	RobotMap.driveTrain.WARlordsDrive(throttle, steering, quickturn);

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
