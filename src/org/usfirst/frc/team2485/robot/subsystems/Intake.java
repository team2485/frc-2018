package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * 
 * @author Brett Kim
 *
 */

public class Intake extends Subsystem {
	private TalonSRX left;
	private TalonSRX right;
	
	public Intake() {
		this.left = RobotMap.armLeftTalon;
		this.right = RobotMap.armRightTalon;
	}

	public void setRollers(double pwm) {
		left.set(ControlMode.PercentOutput, pwm);
		right.set(ControlMode.PercentOutput, pwm);
	}
	
	public void stopRollers() {
		left.set(ControlMode.PercentOutput, 0);
		right.set(ControlMode.PercentOutput, 0);
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
}
