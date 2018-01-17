package org.usfirst.frc.team2485.util;

import org.usfirst.frc.team2485.robot.RobotMap;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.PIDSourceType;

public class DeadReckoning {

	private double x, y;
	private double lastDist;
	private double lastAngle;
	private boolean running;
	private PigeonIMU gyro;
	private double velocity;
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
		velocity = 0;
		gyro.setFusedHeading(0, 0); 
		gyro.setYaw(0, 0); //these lines effectively "reset" the pigeon imu.
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
		short [] xyz = new short [3];
		RobotMap.pigeon.getBiasedAccelerometer(xyz);
//		System.out.println("X"+xyz[0]);
//		System.out.println(xyz[1]);
//		System.out.println(xyz[2]);
		
		double[] ypr = new double[3];
		RobotMap.pigeon.getYawPitchRoll(ypr);
//
//		double acceleration = (xyz[1] * 9.8 * 100 / 16384 / 2.54) - 9.8 * 100 / 2.54 * Math.sin(Math.toRadians(ypr[1]));
//		double deltaDist = velocity * .01 + acceleration / 2 * .0001;
//		velocity += acceleration * .01;
		double angle = Math.toRadians(gyro.getFusedHeading()); //use fused heading to accommodate for the 0.25 degree drift that occurs in 15 seconds.
		double avgAngle = (lastAngle + angle) / 2;
		
		x += deltaDist * Math.sin(avgAngle);
		y += deltaDist * Math.cos(avgAngle);

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
