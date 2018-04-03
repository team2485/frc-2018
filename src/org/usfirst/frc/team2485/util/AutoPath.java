package org.usfirst.frc.team2485.util;

import java.util.ArrayList;
import java.util.Arrays;


/**
 * @author Ben Dorsey
 */

public class AutoPath {
	private static final double MAX_VELOCITY = 1000;
	private static final double MAX_ANG_VEL = .75;
	private static final double MAX_ACCELERATION_LINEAR = 20;
	public class Point {
		public double x, y; 
		public double heading, curvature;
		public double arcLength;
		public double maxSpeed;
		private Point(Pair p) {
			this.x = p.getX();
			this.y = p.getY();
		}
	}
	
	public static class Pair {
		private double x;
		private double y;
		public Pair (double x, double y) {
			this.x = x;
			this.y = y;
		}
		
		public double getX() {
			return x;
		}
		
		public double getY() {
			return y;
		}
		
		
		public static Pair linearBezier(Pair p1, Pair p2, double t) {
			return new Pair(p1.x * (1 - t) + p2.x * t, p1.y * (1 - t) + p2.y * t);
		}
	}
	
	public interface ParameterizedCurve {
		/**
		 * 
		 * @author Ben Dorsey
		 * @param t parameter between 0 and 1
		 * @return point for that parameter
		 * 
		 */
		public Pair getPointForParameter(double t); 
	}
	private Point[] points; 
	private static Pair[] unitClothoid;
	
	public AutoPath(Pair[]... pairs) {
		
		// concat into single array
		ArrayList<Pair> newPairs = new ArrayList<>();
		for (int i = 0; i < pairs.length; i++) {
			newPairs.addAll(Arrays.asList(pairs[i]));
		}
		
		for (int i = 0; i < newPairs.size() - 1; i++) {
			double dX = newPairs.get(i + 1).x - newPairs.get(i).x;
			double dY = newPairs.get(i + 1).y - newPairs.get(i).y;
			if (Math.hypot(dX, dY) < Math.pow(10, -10)) {
				newPairs.remove(i);
				i--;
			}
		}
		
		// convert from pairs to points
		this.points = new Point[newPairs.size()];
		for (int i = 0; i < newPairs.size(); i++) {
			this.points[i] = new Point(newPairs.get(i));
		}
		
		generateCurve();
		
	} 
	
	public static AutoPath getAutoPathForClothoidSpline(Pair[] points, double[] distances) {
		Pair[][] input = new Pair[distances.length * 2 + 1][];
		
		double percent = 1 - distances[0] / Math.hypot(points[1].x - points[0].x, 
				points[1].y - points[0].y);
		double xEnd = (1 - percent) * points[0].x + percent * points[1].x;
		double yEnd = (1 - percent) * points[0].y + percent * points[1].y;
		input[0] = AutoPath.getPointsForBezier(200, points[0], new Pair(xEnd, yEnd));
		
		
		for (int i = 1; i < points.length - 1; i++) {

			double lastX = points[i - 1].x, lastY = points[i - 1].y;
			double thisX = points[i].x, thisY = points[i].y;
			double nextX = points[i + 1].x, nextY = points[i + 1].y;

			input[2 * i - 1] = AutoPath.getPointsForClothoid(new Pair(thisX, thisY), 
					Math.atan2(thisY - lastY, thisX - lastX), Math.atan2(nextY - thisY, nextX - thisX), 
					distances[i - 1]);
			
			double percentStart = distances[i - 1] / Math.hypot(nextX - thisX, nextY - thisY);
			double percentEnd = (i == points.length - 2) ? 1 : 1 - distances[i] / Math.hypot(nextX - thisX, nextY - thisY);
			double xStart = (1 - percentStart) * thisX + percentStart * nextX;
			double yStart = (1 - percentStart) * thisY + percentStart * nextY;
			xEnd = (1 - percentEnd) * thisX + percentEnd * nextX;
			yEnd = (1 - percentEnd) * thisY + percentEnd * nextY;
			
			input[2 * i] = AutoPath.getPointsForBezier(200, new Pair(xStart, yStart), new Pair(xEnd, yEnd));

		}
		
		return new AutoPath(input);
		
	}
	
//	public static Pair[] getPointsForArc(Pair center, double startAngle, double endAngle, double radius) {
//		
//		return getPointsForFunction((double t) -> {
//			double angle = t * endAngle + (1 - t)*startAngle;
//			return new Pair(center.x + , y)
//		}, 10000);
//	}
	
	private void generateCurve() {
		int len = points.length;
		points[0].arcLength = 0;
		for (int i = 0; i < len - 1; i++) {
			double dX = points[i + 1].x - points[i].x;
			double dY = points[i + 1].y - points[i].y;
			points[i].heading = Math.atan2(dX, dY); // this is switched intentionally i swear
			if (points[i].heading < 0) {
				points[i].heading += Math.PI * 2;
			}
			points[i + 1].arcLength = points[i].arcLength + Math.hypot(dX, dY);
		}
		points[len - 1].heading = points[len - 2].heading;
		
		for (int i = 0; i < points.length - 2; i++) {
			double diffHeading = points[i + 1].heading - points[i].heading;
			if (diffHeading > Math.PI) {
				diffHeading -= Math.PI * 2;
			} else if (diffHeading < -Math.PI) {
				diffHeading += Math.PI * 2;
			}
			points[i].curvature = diffHeading / (points[i + 1].arcLength - points[i].arcLength);
		}
		points[len - 1].curvature = points[len - 2].curvature = points[len - 3].curvature;
		
		for (int i = 0; i < points.length; i++) {
			points[i].maxSpeed = MAX_VELOCITY;
			for (int j = i; j < points.length; j++) {
				if (Math.abs(points[j].curvature) > 0 && 
						2 * MAX_ACCELERATION_LINEAR * (points[j].arcLength - points[i].arcLength) < points[i].maxSpeed * points[i].maxSpeed) {
					double maxSpeed = MAX_ANG_VEL / Math.abs(points[j].curvature);
					maxSpeed = Math.sqrt(maxSpeed * maxSpeed + 2 * MAX_ACCELERATION_LINEAR * (points[j].arcLength - points[i].arcLength));
					points[i].maxSpeed = Math.min(points[i].maxSpeed, maxSpeed);
				}
			}
		}
		
	}
	
	public Pair[] getPairs() {
		Pair[] pairs = new Pair[points.length];
		for (int i = 0; i < points.length; i++) {
			pairs[i] = new Pair(points[i].x, points[i].y);
		}
		return pairs;
	}
	
	public Point getPointAtDist(double dist) {
		for (int i = 0; i < points.length; i++) {
			if (dist < points[i].arcLength) {
				return points[i];
			}
		}
		return points[points.length - 1];
	}

	public double getCurvatureAtDist(double dist) {
		return getPointAtDist(dist).curvature;
	}
	
	public double getHeadingAtDist(double dist) {
		return getPointAtDist(dist).heading;
	}
	
	public double getPathLength() {
		return points[points.length - 1].arcLength;
	}
	
	public static Pair[] getPointsForFunction(ParameterizedCurve p, int numPoints) {
		Pair[] points = new Pair[numPoints];
		for (int i = 0; i < numPoints; i++) {
			points[i] = p.getPointForParameter((double)i/numPoints);
		}
		return points;
	}
	
	private static Pair bezier(Pair[] initPoints, double t) {
		Pair[] points = new Pair[initPoints.length - 1];
		for (int i = 0; i < initPoints.length - 1; i++) {
			points[i] = Pair.linearBezier(initPoints[i], initPoints[i + 1], t);
		}
		
		if (points.length == 1) {
			return points[0];
		} else {
			return bezier(points, t);
		}
	}
	
	public static Pair[] getPointsForBezier(int numPoints, Pair... controlPoints) {
		ParameterizedCurve p = (double t) -> {
			return bezier(controlPoints, t);
		};
		return getPointsForFunction(p, numPoints);
	}
	
	public static Pair[] getPointsForUnitClothoid(int numPoints) {
		Pair[] points = new Pair[numPoints];
		points[0] = new Pair(0, 0);
		
		for (int i = 1; i < numPoints; i++) {
			double arcLength = 1.0 * i / numPoints * Math.sqrt(Math.PI);
			double angle = 0.5 * arcLength * arcLength;
			points[i] = new Pair(points[i - 1].x + Math.cos(angle) * 1.0 / numPoints, points[i - 1].y + Math.sin(angle) * 1.0 / numPoints); 
		}
		return points;

	}
	
	public static Pair[] getPointsForClothoid(Pair vertex, double startAngle, double endAngle, double dMax) {
		if (unitClothoid == null) {
			unitClothoid = getPointsForUnitClothoid(2000);
		}
		// calculate angle delta
		double deltaAngle = endAngle - startAngle;
		while (Math.abs(deltaAngle) > Math.PI) {
			if (deltaAngle > 0) {
				deltaAngle -= 2 * Math.PI;
			} else {
				deltaAngle += 2 * Math.PI;	
			}
		}
		
		// dMax for unit clothoid of specified angle
		int pointsUsed = (int) (unitClothoid.length * Math.sqrt(Math.abs(deltaAngle) / Math.PI)); // half if angle is 90 degrees
		Pair midpoint = unitClothoid[pointsUsed];
		double dUnit = midpoint.x + midpoint.y * Math.tan(Math.abs(deltaAngle) / 2);
	
		// transform points
		Pair[] clothoid = new Pair[2 * pointsUsed + 1];
		for (int i = 0; i <= pointsUsed; i++) {
			double x = unitClothoid[i].x, y = unitClothoid[i].y;
			// translate so vertex @ 0, 0
			x -= dUnit;
			// scale
			x *= dMax / dUnit;
			y *= dMax / dUnit;
			// flip if necessary
			if (deltaAngle < 0) {
				y *= -1;
			}
			
			// reflect so have both halves of curve
			double distToLine = x * Math.cos(deltaAngle / 2) + y * Math.sin(deltaAngle/2);
			double x1 = x - 2 * distToLine * Math.cos(deltaAngle/2);
			double y1 = y - 2 * distToLine * Math.sin(deltaAngle/2);
			clothoid[i] = new Pair(x, y);
			clothoid[clothoid.length - 1 - i] = new Pair(x1, y1);

		}
		
		for (int i = 0; i < clothoid.length; i++) {
			double x = clothoid[i].x;
			double y = clothoid[i].y;
			
			//rotate by theta = startAngle
			double tempX = x * Math.cos(startAngle) - y * Math.sin(startAngle);
			double tempY = x * Math.sin(startAngle) + y * Math.cos(startAngle);
			x = tempX;
			y = tempY;
			//translate so vertex @ vertex
			x += vertex.x;
			y += vertex.y;
			
			clothoid[i].x = x;
			clothoid[i].y = y;
		}
		return clothoid;
		
	}
	
	
}
