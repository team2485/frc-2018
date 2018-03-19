package org.usfirst.frc.team2485.util;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

public abstract class WarlordsControlSystem implements PIDOutput {
	
	
	protected PIDOutput[] outputs;
	protected PIDSource[] sources;
	protected double setpoint;

	private boolean enabled = false;

	private long period = DEFAULT_PERIOD;
	private static final long DEFAULT_PERIOD = 10;

	private Timer pidTimer;
	
	protected PIDSource setpointSource;


	public WarlordsControlSystem() {
		pidTimer = new Timer(true);
		pidTimer.schedule(new PIDTask(), 0, period);
		
	}
	
	public synchronized void setSources(PIDSource... sources) {
		this.sources = sources;
	}
	
	public synchronized void setSetpointSource(PIDSource source) {
		this.setpointSource = source;
	}
	
	public synchronized void setOutputs(PIDOutput... outputs) {
		this.outputs = outputs;
	}
	

	/**
	 * @return time between calculations (millis)
	 */
	public long getPeriod() {
		return period;
	}
	
	/**
	 * @param period time between calculations (millis)
	 */
	public void setPeriod(long period) {
		this.period = period;
		pidTimer.cancel();
		pidTimer = new Timer(true);
		pidTimer.schedule(new PIDTask(), 0, period);
	}
	
	/**
	 * @return true if calculate is currently being run
	 */
	public boolean isEnabled() {
		return enabled;
	} 

	/**
	 * makes calculate start running periodically
	 */
	public synchronized void enable() {
		this.enabled = true;
	}

	/**
	 * Disables and clears integral and derivative terms
	 */
	public synchronized void disable() {
		this.enabled = false;
	}
	
	public void setEnabled(boolean enabled) {
		if (enabled) {
			enable();
		} else {
			disable();
		}
	}

	
	/**
	 * Frees all resources related to PID calculations
	 */
	public void finalize() {
		pidTimer.cancel();
		pidTimer = null;
		outputs = null;
		sources = null;
	}
	

	/**
	 * Calculates output based on sensorVal but does not read from source or write to output directly
	 */
	protected abstract void calculate();
	
	@Override
	public void pidWrite(double output) {
		setSetpoint(output);
	}
	
	
	public void setSetpoint(double setpoint) {
		if (setpointSource != null) {
			throw new SettingSetpointWithSetpointSourceSetException();
		}
		this.setpoint = setpoint;
	}
	
	public double getSetpoint() {
		return setpoint;
	}
	private class PIDTask extends TimerTask {

		@Override
		public void run() {
			synchronized (this) {
				if (enabled) {
					if (setpointSource != null) {
						setpoint = setpointSource.pidGet();
					}
					try {
						calculate();
					} catch (Exception e) {
						e.printStackTrace();
						// so that we don't have any more failing silently
					}
				}
			}
			
		}
	}
	
	@SuppressWarnings("serial")
	private class SettingSetpointWithSetpointSourceSetException extends RuntimeException {
		public SettingSetpointWithSetpointSourceSetException() {
			super("Unset the setpoint source before you set the setpoint");
		}
		
	}
}
