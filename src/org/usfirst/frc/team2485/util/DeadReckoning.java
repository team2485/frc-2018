package org.usfirst.frc.team2485.util;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.PIDSourceType;

public class DeadReckoning {

	private double x, y;
	private double lastDist;
	private boolean running;
	private PigeonIMU gyro;
	private TalonSRXEncoderWrapper leftEnc, rightEnc;

	public DeadReckoning(PigeonIMU gyro, TalonSRXEncoderWrapper leftEnc, TalonSRXEncoderWrapper rightEnc) {
		this.gyro = gyro;
		this.rightEnc = rightEnc;
		this.leftEnc = leftEnc;
		this.leftEnc.setPIDSourceType(PIDSourceType.kDisplacement);
		this.rightEnc.setPIDSourceType(PIDSourceType.kDisplacement);
		new UpdateThread().start();
	}

	public void start() {
		this.running = true;
		zero();
	}

	public synchronized void zero() {
		x = 0;
		y = 0;
		gyro.setFusedHeading(0, 50); gyro.setYaw(0, 50); //these lines effectively "reset" the pigeon imu.
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
		double curDist = (leftEnc.pidGet() + rightEnc.pidGet()) / 2;
		double deltaDist = curDist - lastDist;
		double angle = Math.toRadians(gyro.getFusedHeading()); //use fused heading to accommodate for the 0.25 degree drift that occurs in 15 seconds.

		x += deltaDist * Math.sin(angle);
		y += deltaDist * Math.cos(angle);

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
