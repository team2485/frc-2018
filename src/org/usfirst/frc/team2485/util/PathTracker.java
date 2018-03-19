package org.usfirst.frc.team2485.util;

import java.util.Timer;
import java.util.TimerTask;

import org.usfirst.frc.team2485.util.AutoPath.Point;

public class PathTracker {
	
	private DeadReckoning positionTracker;
	private AutoPath path;
	private double pathDist = 0;
	private double drift = 0;
	private static final double PRECISION = 0.25; // inches
	private boolean enabled = false;
	
	public PathTracker(DeadReckoning positionTracker) {
		this.positionTracker = positionTracker;
		new Timer().schedule(new UpdateTask(), 0, 100);
	}
	
	public void start(AutoPath path) {
		this.path = path;
		drift = pathDist = 0;
		enabled = true;
		positionTracker.start();
	}
	

	
	public void stop() {
		enabled = false;
		positionTracker.stop();
	}
	
	public double getDrift() {
		return drift;
	}
	
	public double getPathDist() {
		return pathDist;
	}
	
	private double updateEstimate() {
		// get actual position
		double x = positionTracker.getX();
		double y = positionTracker.getY();
		// get target position
		Point targPoint = path.getPointAtDist(pathDist);
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
		return progress;
	}
	
	private void updateEstimateIterated() {
		while (updateEstimate() > PRECISION);
	}
	
	private class UpdateTask extends TimerTask {
		@Override
		public void run() { 
			if (enabled) {
				updateEstimateIterated();
			}
		}
	}
}
