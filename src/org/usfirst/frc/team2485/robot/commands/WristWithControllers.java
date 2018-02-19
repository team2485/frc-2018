package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.Arm;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class WristWithControllers extends Command {
	private static final double ELBOW_TOLERANCE = 0.0025;
	public static boolean isManual = true;
	
    public WristWithControllers() {
    	requires(RobotMap.arm);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double axis = -ThresholdHandler.deadbandAndScale(OI.operator.getRawAxis(OI.XBOX_LYJOYSTICK_PORT), OI.XBOX_AXIS_DEADBAND, 0, 1); // joystick command

    	double theta2 = RobotMap.arm.getWristAngle();
    	if (axis != 0 && (axis > 0 || theta2 > RobotMap.arm.getMinWristPos())) {
    		isManual = true;
    		RobotMap.arm.setWristManual(axis);
    	} else if (isManual) {
    		if (RobotMap.arm.getElbowAngle() > 0) {
    			RobotMap.arm.setThetaHigh(theta2);
    		} else {
    			RobotMap.arm.setThetaLow(theta2);
    		}
    		isManual = false;
    	}
    	
    	if(Math.abs(RobotMap.arm.getElbowAngle() - RobotMap.arm.getElbowSetpoint()) < ELBOW_TOLERANCE) {
    		RobotMap.arm.setElbowManual(0);
    	} else {
    		RobotMap.arm.setElbowPos(RobotMap.arm.getElbowSetpoint());
    	}
    	
    	if (!isManual) {
    		if (RobotMap.arm.getElbowAngle() * RobotMap.arm.getElbowSetpoint() < 0) { // crossing from low to high
    			RobotMap.arm.setWristPos(0.25);
    		} else if (RobotMap.arm.getElbowAngle() < 0) {
    			RobotMap.arm.setWristPos(Math.max(RobotMap.arm.getThetaLow(), RobotMap.arm.getMinWristPos()));
    		} else {
    			RobotMap.arm.setWristPos(Math.max(RobotMap.arm.getThetaHigh(), RobotMap.arm.getMinWristPos()));
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
