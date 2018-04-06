package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.AutoLogger;
import org.usfirst.frc.team2485.util.Event.Type;

import edu.wpi.first.wpilibj.command.Command;

public class WaitUntilArmUp extends Command {
	public WaitUntilArmUp() {
		requires(RobotMap.intake);
	}

	@Override
	protected boolean isFinished() {
		return Math.abs(RobotMap.arm.getThetaWrist() - RobotMap.wristEncoderWrapperDistance.pidGet()) < .02 && 
				Math.abs(RobotMap.arm.getThetaElbow() - RobotMap.arm.getElbowAngle()) < .03;
	}
	
	@Override
	protected void end() {
		// TODO Auto-generated method stub
		super.end();
		AutoLogger.addEvent(Type.STOP, "WaitUntilArmUp", isFinished() ? "" : "timeout");
	}
	
	
	
}
