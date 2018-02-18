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
	
	public Intake() {
		this.left = RobotMap.intakeLeftTalon;
		this.right = RobotMap.intakeRightTalon;
	}
	
	public void initDefaultCommand() {
	}

	public void setRollers(double pwm) {
		left.set(ControlMode.PercentOutput, pwm);
		right.set(ControlMode.PercentOutput, pwm);
	}
	
	public void stopRollers() {
		left.set(ControlMode.PercentOutput, 0);
		right.set(ControlMode.PercentOutput, 0);
	}

	public boolean hasCube() {
		return false;	
	}
	
	public void updateConstants() {
		RobotMap.intakeLeftTalon.configContinuousCurrentLimit(ConstantsIO.intakeIMax, 0);
		RobotMap.intakeRightTalon.configContinuousCurrentLimit(ConstantsIO.intakeIMax, 0);
		RobotMap.intakeLeftTalon.enableCurrentLimit(true);
		RobotMap.intakeRightTalon.enableCurrentLimit(true);
	}
}
