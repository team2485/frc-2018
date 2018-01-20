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
	public static final double kIntakePWM = 0.5;
	
	public Intake() {
		this.left = RobotMap.armLeftTalon;
		this.right = RobotMap.armRightTalon;
	}

	public void startRollers() {
		left.set(ControlMode.PercentOutput, kIntakePWM);
		right.set(ControlMode.PercentOutput, kIntakePWM);
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
