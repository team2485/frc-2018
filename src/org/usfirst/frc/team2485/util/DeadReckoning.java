package org.usfirst.frc.team2485.util;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;

public class DeadReckoning {

	private double x, y;
	private double lastDist;
	private boolean running;
	private AHRS gyro;
	private Encoder leftEnc, rightEnc;

	public DeadReckoning(AHRS gyro, Encoder leftEnc, Encoder rightEnc) {
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
		gyro.reset();
		leftEnc.reset();
		rightEnc.reset();
		lastDist = 0;
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
		double curDist = (leftEnc.getDistance() + rightEnc.getDistance()) / 2;
		double deltaDist = curDist - lastDist;
		double angle = gyro.getAngle();

		x += deltaDist * Math.sin(Math.toRadians(angle));
		y += deltaDist * Math.cos(Math.toRadians(angle));

		lastDist = curDist;
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
