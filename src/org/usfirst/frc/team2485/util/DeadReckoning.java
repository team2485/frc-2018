package org.usfirst.frc.team2485.util;



public class DeadReckoning {

	private double x, y;
	private double lastDist;
	private double lastAngle;
	private boolean running;
	private PigeonWrapperRateAndAngle gyro;
//	private double velocity;
	private TalonSRXEncoderWrapper leftEnc, rightEnc;

	public DeadReckoning(PigeonWrapperRateAndAngle gyro, TalonSRXEncoderWrapper leftEnc, TalonSRXEncoderWrapper rightEnc) {
		this.gyro = gyro;
		this.rightEnc = rightEnc;
		this.leftEnc = leftEnc;
		new UpdateThread().start();
	}

	public void start() {
		this.running = true;
		zero();
	}

	public synchronized void zero() {
		x = 0;
		y = 0;
//		velocity = 0;
		gyro.reset();
		leftEnc.reset();
		rightEnc.reset();
		lastDist = 0;
		lastAngle = 0;
	}

	public void stop() {
		this.running = false;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	private synchronized void update() {
		double curDist = (leftEnc.pidGet() + rightEnc.pidGet()) / 2;
		double deltaDist = curDist - lastDist;
		

		double[] ypr = new double[3];
		gyro.getPigeon().getYawPitchRoll(ypr);
//
//		double acceleration = (xyz[1] * 9.8 * 100 / 16384 / 2.54) - 9.8 * 100 / 2.54 * FastMath.sin(FastMath.toRadians(ypr[1]));
//		double deltaDist = velocity * .01 + acceleration / 2 * .0001;
//		velocity += acceleration * .01;
		double angle = gyro.pidGet(); //use fused heading to accommodate for the 0.25 degree drift that occurs in 15 seconds.
		double avgAngle = (lastAngle + angle) / 2;
		x += deltaDist * FastMath.sin(avgAngle);
		y += deltaDist * FastMath.cos(avgAngle);

		lastDist = curDist;
		lastAngle = angle;
	}

	private class UpdateThread extends Thread {
		@Override
		public void run() {
			while (true) {
				if (running) {
					update();
				}
				try {
					Thread.sleep(5);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
	}
}
