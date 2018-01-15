package org.usfirst.frc.team2485.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
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
	public static final int driveRightPort2 = 2;
	public static final int driveRightPort3 = 3;
	public static final int driveLeftPort1 = 5;
	public static final int driveLeftPort2 = 6;
	public static final int driveLeftPort3 = 7;
	
	
	public static TalonSRX driveLeftTalon1;
	public static TalonSRX driveLeftTalon2;
	public static TalonSRX driveLeftTalon3;
	public static TalonSRX driveRightTalon1;
	public static TalonSRX driveRightTalon2;
	public static TalonSRX driveRightTalon3;
	public static TalonSRX[] driveTalons;
	
	public static TalonSRXWrapper driveLeftTalonCurrentWrapper1;
	public static TalonSRXWrapper driveLeftTalonCurrentWrapper2;
	public static TalonSRXWrapper driveLeftTalonCurrentWrapper3;
	public static TalonSRXWrapper driveRightTalonCurrentWrapper1;
	public static TalonSRXWrapper driveRightTalonCurrentWrapper2;
	public static TalonSRXWrapper driveRightTalonCurrentWrapper3;
	
	public static TalonSRXWrapper driveLeftTalonPWMWrapper1;
	public static TalonSRXWrapper driveLeftTalonPWMWrapper2;
	public static TalonSRXWrapper driveLeftTalonPWMWrapper3;
	public static TalonSRXWrapper driveRightTalonPWMWrapper1;
	public static TalonSRXWrapper driveRightTalonPWMWrapper2;
	public static TalonSRXWrapper driveRightTalonPWMWrapper3;

	
	public static SpeedControllerWrapper driveLeftCurrent, driveRightCurrent;
	public static SpeedControllerWrapper driveLeftPWM, driveRightPWM;

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
				driveRightTalon1, driveRightTalon2, driveRightTalon3, 
		};
		for(TalonSRX t : driveTalons) {
			t.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		}
		driveLeftTalon2.set(ControlMode.Follower, driveLeftPort1);
		driveLeftTalon3.set(ControlMode.Follower, driveLeftPort1);
		driveRightTalon2.set(ControlMode.Follower, driveRightPort1);
		driveRightTalon3.set(ControlMode.Follower, driveRightPort1);
		driveLeftTalon1.selectProfileSlot(0, 0);
		driveRightTalon1.selectProfileSlot(0, 0);

	
		
//		pigeon = new PigeonIMU(driveRightTalon1);
		

		// Construct Wrappers
		driveLeftTalonCurrentWrapper1 = new TalonSRXWrapper(ControlMode.Current, driveLeftTalon1);
//		driveLeftTalonCurrentWrapper2 = new TalonSRXWrapper(ControlMode.Current, driveLeftTalon2);
//		driveLeftTalonCurrentWrapper3 = new TalonSRXWrapper(ControlMode.Current, driveLeftTalon3);
		driveRightTalonCurrentWrapper1 = new TalonSRXWrapper(ControlMode.Current, driveRightTalon1);
//		driveRightTalonCurrentWrapper2 = new TalonSRXWrapper(ControlMode.Current, driveRightTalon2);
//		driveRightTalonCurrentWrapper3 = new TalonSRXWrapper(ControlMode.Current, driveRightTalon3);
		
		driveLeftTalonPWMWrapper1 = new TalonSRXWrapper(ControlMode.PercentOutput, driveLeftTalon1);
//		driveLeftTalonPWMWrapper2 = new TalonSRXWrapper(ControlMode.PercentOutput, driveLeftTalon2);
//		driveLeftTalonPWMWrapper3 = new TalonSRXWrapper(ControlMode.PercentOutput, driveLeftTalon3);
		driveRightTalonPWMWrapper1 = new TalonSRXWrapper(ControlMode.PercentOutput, driveRightTalon1);
//		driveRightTalonPWMWrapper2 = new TalonSRXWrapper(ControlMode.PercentOutput, driveRightTalon2);
//		driveRightTalonPWMWrapper3 = new TalonSRXWrapper(ControlMode.PercentOutput, driveRightTalon3);


		
		driveLeftCurrent = new SpeedControllerWrapper(driveLeftTalonCurrentWrapper1);
		driveRightCurrent = new SpeedControllerWrapper(driveRightTalonCurrentWrapper1);
		
		driveLeftPWM = new SpeedControllerWrapper(driveLeftTalonPWMWrapper1);
		driveRightPWM = new SpeedControllerWrapper(driveRightTalonPWMWrapper1);
		
		driveLeftCurrent.setInverted(true);
		driveLeftPWM.setInverted(true);
		
//		pigeonRateWrapper = new PigeonWrapperRateAndAngle(PIDSourceType.kRate, Units.RADS);
//		pigeonDisplacementWrapper = new PigeonWrapperRateAndAngle(PIDSourceType.kDisplacement, Units.RADS);
		
		driveLeftEncoderWrapperRate = new TalonSRXEncoderWrapper(driveLeftTalon3, PIDSourceType.kRate);
		driveRightEncoderWrapperRate = new TalonSRXEncoderWrapper(driveRightTalon3, PIDSourceType.kRate);
		driveLeftEncoderWrapperDistance = new TalonSRXEncoderWrapper(driveLeftTalon3,  PIDSourceType.kDisplacement);
		driveRightEncoderWrapperDistance = new TalonSRXEncoderWrapper(driveRightTalon3, PIDSourceType.kDisplacement);
		
		driveLeftTalon3.setSensorPhase(true);
		driveRightTalon3.setSensorPhase(false);
		driveLeftTalon3.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 0);
		driveRightTalon3.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 0);
		driveLeftTalon3.configVelocityMeasurementWindow(1, 0);
		driveRightTalon3.configVelocityMeasurementWindow(1, 0);
		
		// Configure Hardware
		driveLeftEncoderWrapperDistance.setDistancePerRevolution(-WHEEL_RADIUS * 2 * Math.PI * 24.0 / 54);
		driveRightEncoderWrapperDistance.setDistancePerRevolution(WHEEL_RADIUS * 2 * Math.PI * 24.0 / 54);
		driveLeftEncoderWrapperRate.setDistancePerRevolution(-WHEEL_RADIUS * 2 * Math.PI * 24.0 / 54);
		driveRightEncoderWrapperRate.setDistancePerRevolution(WHEEL_RADIUS * 2 * Math.PI * 24.0 / 54);
		
		
		// Construct Subsystems
		drivetrain = new DriveTrain();
	}

	public static void updateConstants() {
		drivetrain.updateConstants();
	}
	
}
