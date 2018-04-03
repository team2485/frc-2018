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
	private boolean setAngle = false;
	private FinishedCondition finishedCondition = FinishedCondition.FALSE_CONDITION;
	
	/**
	 * 
	 * @param dist
	 * @param maxVelocity
	 * @param timeout
	 */
	public DriveStraight(double dist, double maxVelocity, int timeout) {
		this(dist, 0, maxVelocity, timeout);
		setAngle = true;
	}
	
	/**
	 * @param dist
	 * @param angle
	 * @param maxVelocity
	 * @param timeout
	 */
	public DriveStraight(double dist, double angle, double maxVelocity, int timeout) {
		this(dist, angle, maxVelocity, timeout, 0);
	}	
	
	/**
	 * @param dist
	 * @param angle
	 * @param maxVelocity
	 * @param timeout
	 * @param tolerance
	 */
	public DriveStraight(double dist, double angle, double maxVelocity, int timeout, double tolerance) {
		this.dist = dist;
		this.angle = angle;
		this.maxVelocity =  maxVelocity;
		this.timeout = timeout;
		this.tolerance = tolerance;
		requires(RobotMap.driveTrain);
	}
	
	public void setFinishedCondition(FinishedCondition finishedCondition) {
		this.finishedCondition = finishedCondition;
	}
	public void setTolerance(double tolerance) {
		this.tolerance = tolerance;
	}
	@Override
	protected void initialize() {
		super.initialize();
//		RobotMap.driveTrain.zeroEncoders();
		startTime = System.currentTimeMillis();
		RobotMap.driveTrain.angRampRate.setRampRates(100, 100);

		if (setAngle)
			angle = RobotMap.pigeonDisplacementWrapper.pidGet();
	}
	@Override
	protected void execute() {
		super.execute();
		finished = RobotMap.driveTrain.driveTo(dist, maxVelocity, angle, 0, tolerance, Math.PI) ||
				finishedCondition.isFinished();
	}

	@Override
	protected boolean isFinished() {
		return finished || (System.currentTimeMillis() - startTime) > timeout;
	}
	
	@Override
	protected void end() {
		RobotMap.driveTrain.reset();
		super.end();

	}


}
