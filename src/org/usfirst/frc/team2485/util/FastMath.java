package org.usfirst.frc.team2485.util;


public class FastMath {
	public static final int numSamples = 1000;
	public static double[] cos;
	public static double[] acos;
	public static void init() {
		cos = new double[numSamples];
		acos = new double[numSamples];
		for (int i = 0; i < numSamples; i++) {
			cos[i] = Math.cos(Math.PI * 2 * i / numSamples);
			acos[i] = Math.acos(2.0 * i / numSamples - 1);
		}
		
	}
	
	public static double sin(double theta) {
		return cos(Math.PI / 2  - theta); 
	}
	
	public static double cos(double theta) {
		theta %= Math.PI * 2;
		if (theta < 0) {
			theta += Math.PI * 2;
		}
		int index = (int) (theta / 2 / Math.PI * numSamples);
		double rem = theta / 2 / Math.PI * numSamples % 1;
		return cos[index] * (1 - rem) + cos[index + 1] * rem;
	}
	
	public static double tan(double theta) {
		return sin(theta) / cos(theta);
	}
	
	public static double asin(double a) {
		return Math.PI / 2 - acos(a);
		
	}

	public static double acos(double a) {
		if (a >= 1) {
			return 0;
		} else if (a <= -1) {
			return Math.PI;
		} else {
			int index = (int) ((a + 1) / 2 * numSamples);
			double rem = ((a + 1) / 2 * numSamples) % 1;
			return acos[index] * (1 - rem) + acos[index + 1] * rem;
		}
	}
	
	public static double atan(double a) {
		return asin(a) / acos(a);
	}
	
	public static double toRadians(double angle) {
		return (angle/180) * Math.PI;
	}
	
	public static double toDegrees(double rad) {
		return (rad/Math.PI) * 180;
	}
	
	public static double toMeters(double in) {
		return in * 0.0254;
	}

	public static double toKilograms(double lb) {
		return lb * 0.45359237;
	}
}
