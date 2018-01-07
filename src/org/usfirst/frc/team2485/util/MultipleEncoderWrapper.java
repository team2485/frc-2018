package org.usfirst.frc.team2485.util;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * Encoder wrapper that returns the minimum, maximum, or average reading from 2 or more encoders
 * @author Jeremy McCulloch
 *
 */
public class MultipleEncoderWrapper implements PIDSource {
	public enum MultipleEncoderWrapperMode {
		AVERAGE, MIN, MAX
	}

	private MultipleEncoderWrapperMode mode;
	private PIDSourceType pidSource;
	private Encoder[] encoders;

	public MultipleEncoderWrapper(PIDSourceType pidSource, 
			MultipleEncoderWrapperMode mode, Encoder... encoders) {
		this.encoders = encoders;
		this.mode = mode;
		this.pidSource = pidSource;
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		this.pidSource = pidSource;		
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return pidSource;
	}

	@Override
	public double pidGet() {

		double min, max;
		min = max = pidSource == PIDSourceType.kDisplacement ? 
				encoders[0].getDistance() : encoders[0].getRate();
				double sum = 0;

				for (Encoder encoder : encoders) {
					double val = ((pidSource == PIDSourceType.kDisplacement) ? 
							encoder.getDistance() : encoder.getRate());
					min = Math.min(min, val);
					max = Math.max(max, val);
					sum += val;
				}

				switch (mode) {
				case AVERAGE:
					return sum / encoders.length;
				case MIN:
					return min;
				case MAX: 
					return max;
				default:
					break;
				}
				return 0;
	}

}
