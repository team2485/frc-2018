package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class DriveTrain extends Subsystem {
	
	public enum DriveSpeed {
		SLOW_SPEED_RATING, NORMAL_SPEED_RATING;

		public double getSpeedFactor() {

			switch (this) {
			case SLOW_SPEED_RATING:
				return 0.5;
			case NORMAL_SPEED_RATING:
				return 1.0;
			default:
				return 1.0;
			}
		}
	}
	
	private double driveSpeed = DriveSpeed.NORMAL_SPEED_RATING.getSpeedFactor();


	public static final double STEERING_DEADBAND = 0.15;
	public static final double THROTTLE_DEADBAND = 0.05;
	private static final double MIN_CURRENT = 2;
	private static final double MAX_CURRENT = 20;
	
	private boolean isQuickTurn;
	
    public DriveTrain() {
        
    }
    
    public void initDefaultCommand() {
        
    }
    
    public void WARLordsDrive() {
    	
    }
    
    public void setDriveSpeed(DriveSpeed speed) {
		driveSpeed = speed.getSpeedFactor();
	}
    
    public void zeroEncoders() {
		
		RobotMap.driveLeftEncoderWrapperDistance.reset();
		RobotMap.driveRightEncoderWrapperDistance.reset();

	}
    
    public void reset() {
		
		RobotMap.driveLeft.set(0);
		RobotMap.driveRight.set(0);
		
	}
    
    public void setLeftRightCurrent(double l, double r) {
		
		RobotMap.driveLeftTalon1.set(ControlMode.Current, l);
		RobotMap.driveLeftTalon2.set(ControlMode.Current, l);
		RobotMap.driveLeftTalon3.set(ControlMode.Current, l);
		RobotMap.driveRightTalon1.set(ControlMode.Current, r);
		RobotMap.driveRightTalon2.set(ControlMode.Current, r);
		RobotMap.driveRightTalon3.set(ControlMode.Current, r);
	}
    
	public void setLeftRightVelocity(double l, double r) {
		
		RobotMap.driveLeftTalon1.set(ControlMode.Velocity, l);
		RobotMap.driveLeftTalon2.set(ControlMode.Velocity, l);
		RobotMap.driveLeftTalon3.set(ControlMode.Velocity, l);
		RobotMap.driveRightTalon1.set(ControlMode.Velocity, r);
		RobotMap.driveRightTalon2.set(ControlMode.Velocity, r);
		RobotMap.driveRightTalon3.set(ControlMode.Velocity, r);
	}
}
