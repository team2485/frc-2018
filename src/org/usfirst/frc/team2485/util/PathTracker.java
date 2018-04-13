package org.usfirst.frc.team2485.util;

import java.util.Timer;
import java.util.TimerTask;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.AutoPath.Point;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PathTracker {
	
	private DeadReckoning positionTracker;
	private AutoPath path;
	private double pathDist = 0;
	private double drift = 0;
	private static final double PRECISION = 0.25; // inches
	private boolean enabled = false;
	private boolean reverse;
	private static final double kFilter = 0.1;

	
	public PathTracker(DeadReckoning positionTracker) {
		this.positionTracker = positionTracker;
		new Timer().schedule(new UpdateTask(), 0, 100);
	}
	
	public void start(AutoPath path, boolean reverse) {
		this.path = path;
		pathDist = drift = 0;
		pathDist = reverse ? path.getPathLength() : 0;
		enabled = true;
		this.reverse = reverse;
	}
	

	
	public void stop() {
		enabled = false;
	}
	
	public double getDrift() {
		return drift;
	}
	
	public double getPathDist() {
		return pathDist;
	}
	
	private double updateEstimate() {
		pathDist = Math.max(0, pathDist);
		pathDist = Math.min(pathDist, path.getPathLength());
	
		// get actual position
		double x = positionTracker.getX();
		double y = positionTracker.getY();
		// get target position
		Point targPoint = path.getPointAtDistInterp(pathDist);
		// calculate error vector (form target to cur position)
		double deltaX = x - targPoint.x;
		double deltaY = y - targPoint.y;
		// get right normal and tangent vectors
		double angle = targPoint.heading; //in radians
		double tX = FastMath.sin(angle);
		double tY = FastMath.cos(angle);
		double nX = tY;
		double nY = -tX;
		// calculate drift and progress (express error in basis formed by t and n)
		drift = nX * deltaX + nY * deltaY;
		double progress = tX * deltaX + tY * deltaY;
		pathDist += progress;
//		System.out.println("progress" + progress);
		return progress;
	}
	
	public void updateEstimateIterated() {
		int counter = 0;
		long time = System.currentTimeMillis();
		while (Math.abs(updateEstimate()) > PRECISION && pathDist >= 0 && pathDist <= path.getPathLength() && counter < 10) {
			counter++;
		}
		System.out.println("time" + counter);
	}
	
	private class UpdateTask extends TimerTask {

		@Override
		public void run() { 
			if (enabled) {
				updateEstimateIterated();
				double encoderDistance = reverse ? pathDist - path.getPathLength() : pathDist;
				SmartDashboard.putNumber("Encoder dist path", encoderDistance);
				SmartDashboard.putNumber("Encoder offset", encoderDistance - RobotMap.deadReckoning.getLastDist());

				positionTracker.addToEncoderPosition(kFilter * (encoderDistance - RobotMap.deadReckoning.getLastDist()));
			}
		}
	}
}
