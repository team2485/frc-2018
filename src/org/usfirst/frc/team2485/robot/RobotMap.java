package org.usfirst.frc.team2485.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import org.usfirst.frc.team2485.robot.subsystems.DriveTrain;
import org.usfirst.frc.team2485.util.PigeonWrapperRateAndAngle;
import org.usfirst.frc.team2485.util.SpeedControllerWrapper;
import org.usfirst.frc.team2485.util.TalonSRXEncoderWrapper;
import org.usfirst.frc.team2485.util.TalonSRXWrapper;
import org.usfirst.frc.team2485.util.PigeonWrapperRateAndAngle.Units;

import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	public static final double ROBOT_WIDTH = 28;
	public static final double WHEEL_RADIUS = 4;
	
	public static final int driveRightPort1 = 1;
	public static final int driveRightPort2 = 3;
	public static final int driveRightPort3 = 2;
	public static final int driveLeftPort1 = 4;
	public static final int driveLeftPort2 = 6;
	public static final int driveLeftPort3 = 5;
	
	
	public static TalonSRX driveLeftTalon1;
	public static TalonSRX driveLeftTalon2;
	public static TalonSRX driveLeftTalon3;
	public static TalonSRX driveRightTalon1;
	public static TalonSRX driveRightTalon2;
	public static TalonSRX driveRightTalon3;
	public static TalonSRX[] driveTalons;
	
	public static TalonSRXWrapper driveLeftTalonWrapper1;
	public static TalonSRXWrapper driveLeftTalonWrapper2;
	public static TalonSRXWrapper driveLeftTalonWrapper3;
	public static TalonSRXWrapper driveRightTalonWrapper1;
	public static TalonSRXWrapper driveRightTalonWrapper2;
	public static TalonSRXWrapper driveRightTalonWrapper3;
	
	public static SpeedControllerWrapper driveLeft, driveRight;

	public static PigeonIMU pigeon;
	
	public static PigeonWrapperRateAndAngle pigeonRateWrapper;
	public static PigeonWrapperRateAndAngle pigeonDisplacementWrapper;
	
	public static TalonSRXEncoderWrapper driveLeftEncoderWrapperRate;
	public static TalonSRXEncoderWrapper driveRightEncoderWrapperRate;
	public static TalonSRXEncoderWrapper driveLeftEncoderWrapperDistance;
	public static TalonSRXEncoderWrapper driveRightEncoderWrapperDistance;
	
	public static PowerDistributionPanel PDP;
	
	public static DriveTrain drivetrain;
	
	public static void init() {
		
		// Construct Hardware
		PDP = new PowerDistributionPanel();
		
		driveLeftTalon1 = new TalonSRX(driveLeftPort1);
		driveLeftTalon2 = new TalonSRX(driveLeftPort2);
		driveLeftTalon3 = new TalonSRX(driveLeftPort3);
		driveRightTalon1 = new TalonSRX(driveRightPort1);
		driveRightTalon2 = new TalonSRX(driveRightPort2);
		driveRightTalon3 = new TalonSRX(driveRightPort3);
		driveTalons = new TalonSRX[] {
				driveLeftTalon1, driveLeftTalon2, driveLeftTalon3, 
				driveRightTalon1, driveRightTalon1, driveRightTalon1, 
		};
		
		pigeon = new PigeonIMU(driveRightTalon1);

		// Construct Wrappers
		driveLeftTalonWrapper1 = new TalonSRXWrapper(ControlMode.PercentOutput, driveLeftTalon1);
		driveLeftTalonWrapper2 = new TalonSRXWrapper(ControlMode.PercentOutput, driveLeftTalon2);
		driveLeftTalonWrapper3 = new TalonSRXWrapper(ControlMode.PercentOutput, driveLeftTalon3);
		driveRightTalonWrapper1 = new TalonSRXWrapper(ControlMode.PercentOutput, driveRightTalon1);
		driveRightTalonWrapper2 = new TalonSRXWrapper(ControlMode.PercentOutput, driveRightTalon2);
		driveRightTalonWrapper3 = new TalonSRXWrapper(ControlMode.PercentOutput, driveRightTalon3);

		driveLeft = new SpeedControllerWrapper(driveLeftTalonWrapper1, driveLeftTalonWrapper2, driveLeftTalonWrapper3);
		driveRight = new SpeedControllerWrapper(driveRightTalonWrapper1, driveRightTalonWrapper2, driveRightTalonWrapper3);
		
		pigeonRateWrapper = new PigeonWrapperRateAndAngle(PIDSourceType.kRate, Units.RADS);
		pigeonDisplacementWrapper = new PigeonWrapperRateAndAngle(PIDSourceType.kDisplacement, Units.RADS);
		
		driveLeftEncoderWrapperRate = new TalonSRXEncoderWrapper(driveLeftTalon3, PIDSourceType.kRate);
		driveRightEncoderWrapperRate = new TalonSRXEncoderWrapper(driveRightTalon3, PIDSourceType.kRate);
		driveLeftEncoderWrapperDistance = new TalonSRXEncoderWrapper(driveLeftTalon3,  PIDSourceType.kDisplacement);
		driveRightEncoderWrapperDistance = new TalonSRXEncoderWrapper(driveRightTalon3, PIDSourceType.kDisplacement);
		
		
		
		// Configure Hardware
		driveLeftEncoderWrapperDistance.setDistancePerRevolution(WHEEL_RADIUS * 2 * Math.PI);
		driveRightEncoderWrapperDistance.setDistancePerRevolution(WHEEL_RADIUS * 2 * Math.PI);
		driveLeftEncoderWrapperRate.setDistancePerRevolution(WHEEL_RADIUS * 2 * Math.PI);
		driveRightEncoderWrapperRate.setDistancePerRevolution(WHEEL_RADIUS * 2 * Math.PI);

		
		
		// Construct Subsystems
		drivetrain = new DriveTrain();
	}

	public static void updateConstants() {
		drivetrain.updateConstants();
	}
	
}
