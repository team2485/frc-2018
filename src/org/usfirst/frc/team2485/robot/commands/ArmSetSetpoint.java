package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.Arm.ArmSetpoint;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class ArmSetSetpoint extends InstantCommand {
	
	double theta1;
	double theta2;
    public ArmSetSetpoint(ArmSetpoint setpoint) {
        requires(RobotMap.arm);
        theta1 = setpoint.getElbowPos();
        theta2 = setpoint.getWristPos();
        
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	RobotMap.arm.setElbowPos(theta1);
    	if (theta1 > 0) {
    		RobotMap.arm.setThetaHigh(theta2);
    	} else {
    		RobotMap.arm.setThetaLow(theta2);
    	}
    	
    }

}
