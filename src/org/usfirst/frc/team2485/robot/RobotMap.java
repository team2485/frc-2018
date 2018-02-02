package org.usfirst.frc.team2485.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import org.usfirst.frc.team2485.robot.subsystems.DriveTrain;
import org.usfirst.frc.team2485.robot.subsystems.Intake;
import org.usfirst.frc.team2485.util.DeadReckoning;
import org.usfirst.frc.team2485.util.PigeonWrapperRateAndAngle;
import org.usfirst.frc.team2485.util.PigeonWrapperRateAndAngle.Units;
import org.usfirst.frc.team2485.util.SpeedControllerWrapper;
import org.usfirst.frc.team2485.util.TalonSRXEncoderWrapper;
import org.usfirst.frc.team2485.util.TalonSRXWrapper;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	public static final double ROBOT_WIDTH = 28;
	public static final double WHEEL_RADIUS = 2;
	
	public static final int driveRightPortTalon = 1;
	public static final int driveRightPortVictor2 = 11;
	public static final int driveRightPortVictor3 = 21;
	public static final int driveRightPortVictor4 = 31;
	
	public static final int driveLeftPortTalon = 2;
	public static final int driveLeftPortVictor2 = 12;
	public static final int driveLeftPortVictor3 = 22;
	public static final int driveLeftPortVictor4 = 32;
	
	public static final int intakePortLeftTalon = 5;
	public static final int intakePortRightTalon = 6;

	public static final int irSensorPort = 0;

	public static TalonSRX intakeLeftTalon;
	public static TalonSRX intakeRightTalon;
	
	public static TalonSRX driveLeftTalon;
	public static VictorSPX driveLeftVictor2;
	public static VictorSPX driveLeftVictor3;
	public static VictorSPX driveLeftVictor4;
	public static TalonSRX driveRightTalon;
	public static VictorSPX driveRightVictor2;
	public static VictorSPX driveRightVictor3;
	public static VictorSPX driveRightVictor4;
	public static TalonSRX[] driveTalons;
	
	public static TalonSRXWrapper driveLeftTalonCurrentWrapper1;
	public static TalonSRXWrapper driveLeftTalonCurrentWrapper2;
	public static TalonSRXWrapper driveLeftTalonCurrentWrapper3;
	//public static TalonSRXWrapper driveLeftTalonCurrentWrapper4;
	public static TalonSRXWrapper driveRightTalonCurrentWrapper1;
	public static TalonSRXWrapper driveRightTalonCurrentWrapper2;
	public static TalonSRXWrapper driveRightTalonCurrentWrapper3;
	//public static TalonSRXWrapper driveRightTalonCurrentWrapper4;
	
	public static TalonSRXWrapper driveLeftTalonPWMWrapper1;
	public static TalonSRXWrapper driveLeftTalonPWMWrapper2;
	public static TalonSRXWrapper driveLeftTalonPWMWrapper3;
	//public static TalonSRXWrapper driveLeftTalonPWMWrapper4;
	public static TalonSRXWrapper driveRightTalonPWMWrapper1;
	public static TalonSRXWrapper driveRightTalonPWMWrapper2;
	public static TalonSRXWrapper driveRightTalonPWMWrapper3;
	//public static TalonSRXWrapper driveRightTalonPWMWrapper4;

	
	public static SpeedControllerWrapper driveLeftCurrent, driveRightCurrent;
	public static SpeedControllerWrapper driveLeftPWM, driveRightPWM;

	public static PigeonIMU pigeon;
	
//	public static PigeonWrapperRateAndAngle pigeonRateWrapper;
//	public static PigeonWrapperRateAndAngle pigeonDisplacementWrapper;
	
	public static TalonSRXEncoderWrapper driveLeftEncoderWrapperRate;
	public static TalonSRXEncoderWrapper driveRightEncoderWrapperRate;
	public static TalonSRXEncoderWrapper driveLeftEncoderWrapperDistance;
	public static TalonSRXEncoderWrapper driveRightEncoderWrapperDistance;
	
	public static DigitalInput irSensor;
	
	public static DeadReckoning deadReckoning;
	
	public static PowerDistributionPanel PDP;
	
	public static DriveTrain driveTrain;
	public static Intake intake;
	
	public static void init() {
		
		// Construct Hardware
		PDP = new PowerDistributionPanel();
		
		intakeLeftTalon = new TalonSRX(intakePortLeftTalon); //Random port for now.
		intakeRightTalon = new TalonSRX(intakePortRightTalon); //Also random port.
		
		driveLeftTalon = new TalonSRX(driveLeftPortTalon);
		driveLeftVictor2 = new VictorSPX(driveLeftPortVictor2);
		driveLeftVictor3 = new VictorSPX(driveLeftPortVictor3);
		driveLeftVictor4 = new VictorSPX(driveLeftPortVictor4);
		
		driveRightTalon = new TalonSRX(driveRightPortTalon);
		driveRightVictor2 = new VictorSPX(driveRightPortVictor2);
		driveRightVictor3 = new VictorSPX(driveRightPortVictor3);
		driveRightVictor4 = new VictorSPX(driveRightPortVictor4);

		
		driveTalons = new TalonSRX[] {
				driveLeftTalon, 
				driveRightTalon
		};

		
		
		
		// Configure Hardware

		for(TalonSRX t : driveTalons) {
			t.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		}
		
//		armLeftTalon.set(ControlMode.Follower, 8);
//		armRightTalon.set(ControlMode.Follower, 9);
		
		driveLeftVictor2.follow(driveLeftTalon);
		driveLeftVictor3.follow(driveLeftTalon);
		driveLeftVictor4.follow(driveLeftTalon);
		
		driveRightVictor2.follow(driveRightTalon);
		driveRightVictor3.follow(driveRightTalon);
		driveRightVictor4.follow(driveRightTalon);
		
		
//		pigeon = new PigeonIMU(driveRightTalon);
		irSensor = new DigitalInput(irSensorPort);

		// Construct Wrappers
		driveLeftTalonCurrentWrapper1 = new TalonSRXWrapper(ControlMode.Current, driveLeftTalon);
		driveRightTalonCurrentWrapper1 = new TalonSRXWrapper(ControlMode.Current, driveRightTalon);

		driveLeftTalonPWMWrapper1 = new TalonSRXWrapper(ControlMode.PercentOutput, driveLeftTalon);
		driveRightTalonPWMWrapper1 = new TalonSRXWrapper(ControlMode.PercentOutput, driveRightTalon);


		
		driveLeftCurrent = new SpeedControllerWrapper(driveLeftTalonCurrentWrapper1);
		driveRightCurrent = new SpeedControllerWrapper(driveRightTalonCurrentWrapper1);
		driveLeftPWM = new SpeedControllerWrapper(driveLeftTalonPWMWrapper1);
		driveRightPWM = new SpeedControllerWrapper(driveRightTalonPWMWrapper1);
		
		driveLeftCurrent.setInverted(true);
		driveLeftPWM.setInverted(true);
		
//		pigeonRateWrapper = new PigeonWrapperRateAndAngle(PIDSourceType.kRate, Units.RADS);
//		pigeonDisplacementWrapper = new PigeonWrapperRateAndAngle(PIDSourceType.kDisplacement, Units.RADS);
		
		driveLeftEncoderWrapperRate = new TalonSRXEncoderWrapper(driveLeftTalon, PIDSourceType.kRate);
		driveRightEncoderWrapperRate = new TalonSRXEncoderWrapper(driveRightTalon, PIDSourceType.kRate);
		driveLeftEncoderWrapperDistance = new TalonSRXEncoderWrapper(driveLeftTalon,  PIDSourceType.kDisplacement);
		driveRightEncoderWrapperDistance = new TalonSRXEncoderWrapper(driveRightTalon, PIDSourceType.kDisplacement);
		
		driveRightVictor3.setSensorPhase(false);
		driveRightVictor3.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 0);
		driveRightVictor3.configVelocityMeasurementWindow(1, 0);
		
		deadReckoning = new DeadReckoning(pigeon, driveLeftEncoderWrapperDistance, driveRightEncoderWrapperDistance);
		
		// Configure Hardware
		driveLeftEncoderWrapperDistance.setDistancePerRevolution(-WHEEL_RADIUS * 2 * Math.PI * 24.0 / 54);
		driveRightEncoderWrapperDistance.setDistancePerRevolution(WHEEL_RADIUS * 2 * Math.PI * 24.0 / 54);
		driveLeftEncoderWrapperRate.setDistancePerRevolution(-WHEEL_RADIUS * 2 * Math.PI * 24.0 / 54);
		driveRightEncoderWrapperRate.setDistancePerRevolution(WHEEL_RADIUS * 2 * Math.PI * 24.0 / 54);
		// Construct Subsystems
		driveTrain = new DriveTrain();
		intake = new Intake();
	}

	public static void updateConstants() {
		driveTrain.updateConstants();
	}
	
}
