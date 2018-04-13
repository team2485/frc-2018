package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.AutoLogger;
import org.usfirst.frc.team2485.util.Event.Type;

import edu.wpi.first.wpilibj.command.Command;

public class WaitUntilClose extends Command{
	private double close;
	
	public WaitUntilClose(double close) {
//		requires(RobotMap.arm);
		this.close = close;
	}
	
	protected void initialize() {
		System.out.println("WaitUntilClose");
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return Math.abs(RobotMap.driveTrain.distancePID.getSetpoint() - RobotMap.driveTrain.getAverageEncoderDistance()) < close;
	}
	
	@Override
	protected void end() {
		// TODO Auto-generated method stub
		super.end();
		
			AutoLogger.addEvent(Type.STOP, "WaitUntilClose", "");
		}
	}


