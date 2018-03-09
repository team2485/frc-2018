package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.FinishedCondition;

import edu.wpi.first.wpilibj.command.Command;

public class DriveTo extends Command{
	private AutoPath path;
	private double maxVelocity;
	private boolean finished, reverse;
	private long startTime;
	private int timeout;
	private double distTolerance;
	private double angleTolerance;
	private boolean variableVMax;
	private FinishedCondition finishedCondition = FinishedCondition.FALSE_CONDITION;
	public DriveTo(AutoPath path, double maxVelocity, boolean reverse, int timeout, boolean variableVMax) {
		this.path = path;
		this.maxVelocity =  maxVelocity;
		this.reverse = reverse;
		this.timeout = timeout;
		this.distTolerance = 7;
		this.angleTolerance = 0.08;
		this.variableVMax = variableVMax;
		setInterruptible(true);
		requires(RobotMap.driveTrain);
	}
	
	public void setFinishedCondition(FinishedCondition finishedCondition) {
		this.finishedCondition = finishedCondition;
	}
	
	public void setDistTolerance (double tolerance) {
		this.distTolerance = tolerance;
	}
	
	public void setAngleTolerance (double tolerance) {
		this.angleTolerance = tolerance;
	}
	
	@Override
	protected void initialize() {
		super.initialize();
		startTime = System.currentTimeMillis();
		RobotMap.driveTrain.zeroEncoders();
		RobotMap.driveTrain.angularVelocityRampRate.setRampRates(100, 100);

	}
	@Override
	protected void execute() {

		double arcLength = RobotMap.driveTrain.getAverageEncoderDistance(), 
				pathLength = path.getPathLength();
		
		if (reverse) {
			arcLength = pathLength + arcLength;
			pathLength *= -1;
		}
		double currentMaxSpeed = maxVelocity;
		
		if (variableVMax) {
			currentMaxSpeed = Math.min(maxVelocity, path.getPointAtDist(RobotMap.driveTrain.getAverageEncoderDistance()).maxSpeed);
		}
		
		finished = RobotMap.driveTrain.driveTo(pathLength, currentMaxSpeed, 
				path.getHeadingAtDist(arcLength), path.getCurvatureAtDist(arcLength), distTolerance, angleTolerance) ||
				finishedCondition.isFinished();
		
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
}
