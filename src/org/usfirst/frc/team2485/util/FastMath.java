package org.usfirst.frc.team2485.util;



public class FastMath {
	public static final int numSamples = 100000;
	public static double[] cos;
	public static double[] acos;
	public static double[] sqrt;
	public static double[] atan;
	public static void init() {
		cos = new double[numSamples + 1];
		acos = new double[numSamples + 1];
		sqrt = new double[numSamples + 1];
		atan = new double[numSamples + 1];
		for (int i = 0; i <= numSamples; i++) {
			cos[i] = Math.cos(Math.PI * 2 * i / numSamples);
			acos[i] = Math.acos(2.0 * i / numSamples - 1);
			sqrt[i] = Math.sqrt(1 + 3.0 * i / numSamples);
			atan[i] = Math.atan(1.0 * i / numSamples);
		}
		
	}
	
	public static double sin(double theta) {
		return cos(Math.PI / 2  - theta); 
	}
	
	public static double cos(double theta) {
		int index = (int) (theta / 2 / Math.PI * numSamples + .5);

		index %= numSamples;
		if (index < 0) {
			index += numSamples;
		}
		return cos[index];
	}
	
	public static double tan(double theta) {
		return sin(theta) / cos(theta);
	}
	
	public static double asin(double a) {
		return Math.PI / 2 - acos(a);
		
	}
	
	public static double atan(double a) {
		if (a < 0) {
			return -atan(-a);
		} else if (a > 1) {
			return Math.PI / 2 - atan(1 / a);
		} else {
			int index = (int) (a * numSamples + .5);
			return atan[index];
		}
	}

	public static double atan2(double y, double x) {
		if (x == 0) {
			return y > 0 ? Math.PI / 2 : 3 * Math.PI / 2;
		} else {
			return x > 0 ? (atan(y / x) + 2 * Math.PI) % (2 * Math.PI): Math.PI + atan(y / x);
		}
	}
	public static double acos(double a) {
		if (a >= 1) {
			return 0;
		} else if (a <= -1) {
			return Math.PI;
		} else {
			int index = (int) ((a + 1) / 2 * numSamples + .5);
			return acos[index];
		}
	}
	
	public static double sqrt(double a) {
		return Math.sqrt(a);
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
	
	public static double hypot(double x, double y) {
		return sqrt(x * x + y * y);
	}
}
