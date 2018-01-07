package org.usfirst.frc.team2485.util;

import java.util.LinkedList;
import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * @author Jeremy McCulloch
 * @author Nicholas Contreras
 */
public class LidarWrapper extends SensorBase implements PIDSource {

	private I2C m_i2c;
	private PIDSourceType pidSourceType;

	private LinkedList<Double> rateMeasurements;

	private static final int ROLLING_AVG_SAMPLE_SIZE = 25;
	private static final int ROLLING_AVG_SAMPLE_RATE = 5;

	public LidarWrapper(Port port) {
		m_i2c = new I2C(port, 0x62);
		pidSourceType = PIDSourceType.kDisplacement;
	}

	public int prepLidarForRateMesaurement() {

		rateMeasurements = new LinkedList<Double>();

		new Timer(true).scheduleAtFixedRate(new TimerTask() {

			@Override
			public void run() {
				updateSpeedRollingAvg();
			}
		}, 0, ROLLING_AVG_SAMPLE_RATE);

		return ROLLING_AVG_SAMPLE_SIZE * ROLLING_AVG_SAMPLE_RATE;

	}

	private void updateSpeedRollingAvg() {

		// if (Math.abs(rate) > 100) {
		// return;
		// }
		//
		// rateMeasurements.add(rate);

		if (rateMeasurements.size() > ROLLING_AVG_SAMPLE_SIZE) {
			rateMeasurements.remove();
		}
	}

	/**
	 * @return distance in inches
	 */
	public double getDistance() {

		byte[] buffer;
		buffer = new byte[2];

		buffer[0] = 0x00;
		buffer[1] = 0x00;

		m_i2c.write(0x00, 0x04);

		try {
			read(0x8f, buffer, 2);
		} catch (BadLidarDataException e) {
			return 0;
		}
		double dist = (Integer.toUnsignedLong(buffer[0] << 8) + Byte
				.toUnsignedInt(buffer[1])) / 2.54;
		
		return dist;
	}

	/**
	 * @return rate in inches / s
	 */
	public double getRate() {

		if (rateMeasurements == null) {
			throw new BadLidarDataException(
					"You must call prepLidarForRateMeasurement() WELL BEFORE you attempt to record rate");
		}

		byte[] buffer;
		buffer = new byte[1];

		buffer[0] = 0x00;

		m_i2c.write(0x04, 0xa0);
		m_i2c.write(0x00, 0x04);

		read(0x09, buffer, 1);

		return 10 * Integer.toUnsignedLong(buffer[0]) / 2.54;

	}

	private void read(int register, byte[] buffer, int count) {
		int busyFlag = 0;
		int busyCounter = 0;

		while (busyFlag != 0) {
			byte[] testSignal = { 0x1 };
			boolean nack = m_i2c.writeBulk(testSignal);

			if (nack) {
				throw new BadLidarDataException(
						"WriteBulk failed to write (in bulk): " + testSignal);
			}

			byte testBuffer[] = new byte[1];
			m_i2c.readOnly(testBuffer, 1);
			busyFlag = testBuffer[0];

			busyCounter++;
			if (busyCounter > 9999) {
				throw new BadLidarDataException("Lidar was too busy: "
						+ busyFlag);
			}
		}

		if (busyFlag == 0) {
			byte[] registerSignal = { (byte) register };
			boolean nack = m_i2c.writeBulk(registerSignal);
			if (nack) {
				throw new BadLidarDataException(
						"Unable to write (bulk) register signal: "
								+ registerSignal);
			}
			m_i2c.readOnly(buffer, count);
		}

		if (busyCounter > 9999) {
			throw new BadLidarDataException("Lidar was too busy: " + busyFlag);
		}

	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return pidSourceType;
	}

	@Override
	public double pidGet() {
		return pidSourceType == PIDSourceType.kDisplacement ? getDistance()
				: getRate();
	}

	@Override
	public void setPIDSourceType(PIDSourceType arg0) {
		pidSourceType = arg0;
	}

	@SuppressWarnings("serial")
	class BadLidarDataException extends RuntimeException {

		public BadLidarDataException(String message) {
			super(message);
		}
	}
}
