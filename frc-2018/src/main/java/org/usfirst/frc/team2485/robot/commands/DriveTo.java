package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.AutoLogger;
import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.AutoPath.Pair;
import org.usfirst.frc.team2485.util.AutoPath.Point;
import org.usfirst.frc.team2485.util.Event.Type;
import org.usfirst.frc.team2485.util.FinishedCondition;

import edu.wpi.first.wpilibj.command.Command;

public class DriveTo extends Command{
	private AutoPath path;
	private double maxVelocity;
	private boolean finished; 
	private boolean reverse;
	private long startTime;
	private int timeout;
	private double distTolerance;
	private double angleTolerance;
	private boolean variableVMax;
	private boolean isDeadReckoning;
	private FinishedCondition finishedCondition = FinishedCondition.FALSE_CONDITION;
	public DriveTo(AutoPath path, double maxVelocity, boolean reverse, int timeout, boolean variableVMax, boolean isDeadReckoning) {
		this.path = path;
		this.maxVelocity =  maxVelocity;
		this.reverse = reverse;
		this.timeout = timeout;
		this.distTolerance = 7;
		this.angleTolerance = 0.08;
		this.variableVMax = variableVMax;
		this.isDeadReckoning = isDeadReckoning;
		setInterruptible(true);
		requires(RobotMap.driveTrain);
	}
	
//	public DriveTo(double dist, double maxVelocity, int timeout, boolean reverse) {
//		this(dist, 0, maxVelocity, timeout);
//	}
	
//	public DriveTo(double dist, double angle, double maxVelocity, int timeout, boolean rev) {
//		this(dist, angle, maxVelocity, timeout, 0);
//	}
	
	public DriveTo(double dist, double angle, double maxVelocity, int timeout, boolean reverse, boolean isDeadReckoning) {
		this(new AutoPath(AutoPath.getPointsForBezier(1000, new Pair(0, 0), new Pair(dist * Math.sin(angle), dist * Math.cos(angle)))),
				maxVelocity, reverse, timeout, false, isDeadReckoning);
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
		System.out.println("DriveTo");
		startTime = System.currentTimeMillis();
//		RobotMap.driveTrain.zeroEncoders();
		RobotMap.driveTrain.angRampRate.setRampRates(100, 100);
		AutoLogger.addEvent(Type.START, "DriveTo", "");
		Point p = reverse ? path.getPointAtDist(path.getPathLength()) : path.getPointAtDist(0);
		RobotMap.deadReckoning.reset(RobotMap.deadReckoning.getX() + p.x, RobotMap.deadReckoning.getY() + p.y);
		RobotMap.deadReckoning.setEncoderPosition(0);
		RobotMap.pathTracker.start(path, reverse);
		RobotMap.pathTracker.updateEstimateIterated();
		double arcLength = RobotMap.pathTracker.getPathDist();
		if (reverse) {
			arcLength -= path.getPathLength();
		}
		RobotMap.deadReckoning.setEncoderPosition(arcLength);
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
			currentMaxSpeed = Math.min(maxVelocity, path.getPointAtDist(arcLength).maxSpeed);
			if (RobotMap.arm.getElbowSetpoint() > 0) {
				currentMaxSpeed /= 2;
			}
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
	protected void end() {
		if (finishedCondition.isFinished()) {
			AutoLogger.addEvent(Type.STOP, "DriveTo", finished ? "finished condition" : "timeout");
		} else {
			AutoLogger.addEvent(Type.STOP, "DriveTo", finished ? "" : "timeout");
		}
		if (isDeadReckoning) {
			Point p = reverse ? path.getPointAtDist(0) : path.getPointAtDist(path.getPathLength());
			RobotMap.deadReckoning.reset(RobotMap.deadReckoning.getX() - p.x, RobotMap.deadReckoning.getY() - p.y);
		} else {
			Point p = reverse ? path.getPointAtDist(path.getPathLength()) : path.getPointAtDist(0);
			RobotMap.deadReckoning.reset(RobotMap.deadReckoning.getX() - p.x, RobotMap.deadReckoning.getY() - p.y);
		}	
		
		RobotMap.pathTracker.stop();
		
	}

	@Override
	protected boolean isFinished() {
		return (finished || (System.currentTimeMillis() - startTime) > timeout) && (System.currentTimeMillis() - startTime) > 50;
	}
}
