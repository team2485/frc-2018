package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
//import org.usfirst.frc.team2485.robot.subsystems.DriveTrain;
import org.usfirst.frc.team2485.util.FinishedCondition;

import edu.wpi.first.wpilibj.command.Command;

public class DriveStraight extends Command{
	
	private double dist, angle;
	private double maxVelocity;
	private boolean finished;
	private int timeout;
	private long startTime;
	private double tolerance;
	//private boolean setAngle = false;
	private FinishedCondition finishedCondition = FinishedCondition.FALSE_CONDITION;
	
	public DriveStraight(double dist, double maxVelocity, int timeout) {
		this(dist, 0, maxVelocity, timeout);
		//setAngle = true;
	}
	
	public DriveStraight(double dist, double angle, double maxVelocity, int timeout) {
		this(dist, angle, maxVelocity, timeout, 0);
	}	
	
	public DriveStraight(double dist, double angle, double maxVelocity, int timeout, double tolerance) {
		this.dist = dist;
		this.angle = angle;
		this.maxVelocity =  maxVelocity;
		this.timeout = timeout;
		this.tolerance = tolerance;
		requires(RobotMap.drivetrain);
	}
	
	public void setFinishedCondition(FinishedCondition finishedCondition) {
		this.finishedCondition = finishedCondition;
	}
	
	@Override
	protected void initialize() {
		super.initialize();
		RobotMap.drivetrain.zeroEncoders();
		startTime = System.currentTimeMillis();
		//if (setAngle)
		//	angle = RobotMap.pigeon.getAngle();
	}
	@Override
	protected void execute() {
		super.execute();
		finished = RobotMap.drivetrain.driveTo(dist, maxVelocity, angle, 0, tolerance) ||
				finishedCondition.isFinished();
	}

	@Override
	protected boolean isFinished() {
		return finished || (System.currentTimeMillis() - startTime) > timeout;
	}
	
	@Override
	protected void end() {
		RobotMap.drivetrain.reset();
		super.end();

	}


}