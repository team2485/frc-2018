package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.Arm;
import org.usfirst.frc.team2485.robot.subsystems.Arm.ArmSetpoint;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmSetSetpoint extends Command {
	
	double theta1;
	double theta2;
    public ArmSetSetpoint(ArmSetpoint setpoint) {
        requires(RobotMap.arm);
        theta1 = setpoint.getElbowPos();
        theta2 = setpoint.getWristPos();
        
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Arm arm = RobotMap.arm;
    	
    	arm.setElbowPos(theta1);
    	if (theta1 > arm.getWristAngle()) {
    		arm.setThetaHigh(theta2);
    	} else {
    		arm.setThetaLow(theta2);
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
