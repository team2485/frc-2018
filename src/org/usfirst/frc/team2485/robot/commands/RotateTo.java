package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.AutoLogger;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.Event.Type;

import edu.wpi.first.wpilibj.command.Command;

public class RotateTo extends Command {
	private double angle;
	private int timeout;
	private boolean finished;
	private double tolerance = .08;
	private long startTime;
	public RotateTo(double angle, int timeout) {
		this.angle = angle;
		this.timeout = timeout;
		setInterruptible(true);
		requires(RobotMap.driveTrain);
	}
	
	public void setTolerance(double tolerance) {
		this.tolerance = tolerance;
	}
	
	
	
	@Override
	protected void initialize() {
		super.initialize();
		startTime = System.currentTimeMillis();
		RobotMap.driveTrain.zeroEncoders();
		RobotMap.driveTrain.angRampRate.setRampRates(ConstantsIO.kUpRamp_AngCurrent, 100);
	}
	@Override
	protected void execute() {
		finished = RobotMap.driveTrain.driveTo(0, 100, 
				angle, 0, 5, tolerance);
		
	}
	
	@Override
	protected void interrupted() {
		finished = true;
	}
	
	@Override
	public synchronized void cancel() {
		finished = true;
	}

	@Override
	protected boolean isFinished() {
		
		return finished || (System.currentTimeMillis() - startTime) > timeout;
	}
	
	@Override
	protected void end() {
		// TODO Auto-generated method stub
		super.end();
			AutoLogger.addEvent(Type.STOP, "RotateTo", finished ? "" : "timeout");
		}
	}

