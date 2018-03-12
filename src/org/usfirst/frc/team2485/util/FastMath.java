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
	
	
}
