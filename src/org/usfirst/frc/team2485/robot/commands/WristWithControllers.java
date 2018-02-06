package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.Arm;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import com.sun.awt.SecurityWarning;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class WristWithControllers extends Command {

    public WristWithControllers() {
    	requires(RobotMap.arm);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	boolean isManual = false;
    	Arm arm = RobotMap.arm;
    	double axis = ThresholdHandler.deadbandAndScale(OI.OPERATOR.getRawAxis(OI.XBOX_LYJOYSTICK_PORT), OI.XBOX_DEADBAND, -1, 1);
    	
    	double theta = arm.getWristAngle();
    	
    	if (Math.abs(axis) < Arm.CRITICAL_ANGLE && Math.abs(arm.getWristAngle()) > arm.getMinWristPos() ) {
    		isManual = true;
    		arm.setWristManual(axis);
    	} else if (!arm.wristPIDisEnabled()) {
    		if (arm.getElbowAngle() > theta) {
    			arm.setThetaHigh(theta);
    		} else {
    			arm.setThetaLow(theta);
    		}
    	}
    	
    	if (!isManual) {
    		if (arm.getElbowAngle() < theta) {
    			arm.setWristPos(Math.max(arm.getThetaLow(), arm.getMinWristPos()));
    		} else {
    			arm.setWristPos(Math.max(arm.getThetaLow(), arm.getMinWristPos()));
    		}
    	}
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
