package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ConstantsIO;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * 
 * @author Brett Kim
 *
 */

public class Intake extends Subsystem {
	public TalonSRX left;
	public TalonSRX right;
	
	public double maxCurrentSteady = 20; 
	public double maxCurrentPeak = 30;
	public boolean overCurrentSteady = false;
	public boolean overCurrentPeak = false;
	public double startSpikeTimeSteady;
	public double startSpikeTimePeak;
	public boolean cubeIntaken = false;
	public boolean ejectingLong = false;
	public final int STEADY_TIME = 1000;
	public final int PEAK_TIME = 250;
	
	public Intake() {
		this.left = RobotMap.intakeLeftTalon;
		this.right = RobotMap.intakeRightTalon;
	}
	
	public void initDefaultCommand() {
	}

	
	public boolean isIntaken() {
		double averageCurrent = (RobotMap.intakeLeftTalon.getOutputCurrent() + RobotMap.intakeRightTalon.getOutputCurrent())/2;
		if (averageCurrent >= maxCurrentSteady && !overCurrentSteady) {
			startSpikeTimeSteady = System.currentTimeMillis();
			overCurrentSteady = true;
		} else if (averageCurrent < maxCurrentSteady) {
			overCurrentSteady = false;
		}
		if (overCurrentSteady && (System.currentTimeMillis() - startSpikeTimeSteady) > STEADY_TIME) {
			cubeIntaken = true;
		}
		
		if (averageCurrent >= maxCurrentPeak && !overCurrentPeak) {
			startSpikeTimePeak = System.currentTimeMillis();
			overCurrentPeak = true;
		} else if (averageCurrent < maxCurrentPeak) {
			overCurrentPeak = false;
		}
		if (overCurrentPeak && (System.currentTimeMillis() - startSpikeTimePeak) > PEAK_TIME) {
			cubeIntaken = true;
		}
		
		return cubeIntaken;
	} 
	
	public void setRollers(double pwm) {
		left.set(ControlMode.PercentOutput, pwm);
		right.set(ControlMode.PercentOutput, -pwm);
	}
	
	public double getRollers() {
		return left.getMotorOutputPercent();
	}
	
	public void stopRollers() {
		left.set(ControlMode.PercentOutput, 0);
		right.set(ControlMode.PercentOutput, 0);
	}

	public boolean hasCube() {
		return !RobotMap.irSensor.get();	
	}
	
	public void updateConstants() {
		RobotMap.intakeLeftTalon.configContinuousCurrentLimit(ConstantsIO.intakeIMax, 0);
		RobotMap.intakeRightTalon.configContinuousCurrentLimit(ConstantsIO.intakeIMax, 0);
		RobotMap.intakeLeftTalon.enableCurrentLimit(true);
		RobotMap.intakeRightTalon.enableCurrentLimit(true);
	}
}
