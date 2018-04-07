package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.Arm.ArmSetpoint;
import org.usfirst.frc.team2485.util.AutoLogger;
import org.usfirst.frc.team2485.util.Event.Type;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class ArmSetSetpoint extends InstantCommand {
	
	double theta1;
	double theta2;
	boolean isClimb;
    public ArmSetSetpoint(ArmSetpoint setpoint) {
//        requires(RobotMap.arm);
        theta1 = setpoint.getElbowPos();
        theta2 = setpoint.getWristPos();
        isClimb = setpoint.equals(ArmSetpoint.CLIMB);
        
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	WristWithControllers.isManual = false;
    	RobotMap.arm.setThetaElbow(theta1);
    	RobotMap.arm.setThetaWrist(theta2 - theta1);
    	
    	 if (isClimb) {
         	RobotMap.arm.setIsClimb(true);
         }
    	 AutoLogger.addEvent(Type.START, "ArmSetSetpoint", ""); // instant
    }

}
