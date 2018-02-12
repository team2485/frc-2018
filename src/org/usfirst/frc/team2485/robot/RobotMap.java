package org.usfirst.frc.team2485.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import org.usfirst.frc.team2485.robot.subsystems.Arm;
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
	
//PORTS - all temporary
	public static final double ROBOT_WIDTH = 28;
	public static final double WHEEL_RADIUS = 2;

	public static final int intakeLeftPort = 9; // Temporary
	public static final int intakeRightPort = 10;
	
	public static final int elbowPort1 = 11; //Temporary
	public static final int elbowPort2 = 12;
	public static final int wristPort = 13;
	
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

//TALONS
	public static TalonSRX intakeLeftTalon;
	public static TalonSRX intakeRightTalon;
	
	public static TalonSRX elbowTalon;
	public static VictorSPX elbowVictor;
	public static TalonSRX wristTalon;
	
	public static TalonSRX[] elbowTalons;

	
	public static TalonSRX driveLeftTalon;
	public static VictorSPX driveLeftVictor2;
	public static VictorSPX driveLeftVictor3;
	public static VictorSPX driveLeftVictor4;
	public static TalonSRX driveRightTalon;
	public static VictorSPX driveRightVictor2;
	public static VictorSPX driveRightVictor3;
	public static VictorSPX driveRightVictor4;
	public static TalonSRX[] driveTalons;
	
//WRAPPERS	
	public static TalonSRXWrapper intakeLeftWrapper;
	public static TalonSRXWrapper intakeRightWrapper;
	
	public static TalonSRXWrapper elbowTalonWrapper1;
	public static TalonSRXWrapper elbowTalonWrapper2;
	public static TalonSRXWrapper wristTalonWrapper;
	public static SpeedControllerWrapper elbowCurrentWrapper;
	public static SpeedControllerWrapper wristCurrentWrapper;
	
	public static TalonSRXWrapper driveLeftTalonCurrentWrapper1;
	public static TalonSRXWrapper driveLeftTalonCurrentWrapper2;
	public static TalonSRXWrapper driveLeftTalonCurrentWrapper3;
	public static TalonSRXWrapper driveLeftTalonCurrentWrapper4;
	public static TalonSRXWrapper driveRightTalonCurrentWrapper1;
	public static TalonSRXWrapper driveRightTalonCurrentWrapper2;
	public static TalonSRXWrapper driveRightTalonCurrentWrapper3;
	public static TalonSRXWrapper driveRightTalonCurrentWrapper4;
	
	public static TalonSRXWrapper driveLeftTalonPWMWrapper1;
	public static TalonSRXWrapper driveLeftTalonPWMWrapper2;
	public static TalonSRXWrapper driveLeftTalonPWMWrapper3;
	public static TalonSRXWrapper driveLeftTalonPWMWrapper4;
	public static TalonSRXWrapper driveRightTalonPWMWrapper1;
	public static TalonSRXWrapper driveRightTalonPWMWrapper2;
	public static TalonSRXWrapper driveRightTalonPWMWrapper3;
	public static TalonSRXWrapper driveRightTalonPWMWrapper4;
	
	public static SpeedControllerWrapper driveLeftCurrent, driveRightCurrent;
	public static SpeedControllerWrapper driveLeftPWM, driveRightPWM;

//SENSORS
	public static PigeonIMU pigeon;
	
	public static PigeonWrapperRateAndAngle pigeonRateWrapper;
	public static PigeonWrapperRateAndAngle pigeonDisplacementWrapper;
	
	public static TalonSRXEncoderWrapper elbowEncoderWrapperDistance;
	public static TalonSRXEncoderWrapper wristEncoderWrapperDistance;
	public static TalonSRXEncoderWrapper elbowEncoderWrapperRate;
	public static TalonSRXEncoderWrapper wristEncoderWrapperRate;
	
	public static TalonSRXEncoderWrapper driveLeftEncoderWrapperRate;
	public static TalonSRXEncoderWrapper driveRightEncoderWrapperRate;
	public static TalonSRXEncoderWrapper driveLeftEncoderWrapperDistance;
	public static TalonSRXEncoderWrapper driveRightEncoderWrapperDistance;
	
	public static DigitalInput irSensor;
	
	public static DeadReckoning deadReckoning;
	
	public static PowerDistributionPanel PDP;

//SUBSYSTEMS
	public static DriveTrain driveTrain;
	public static Intake intake;
	public static Arm arm;

	public static void init() {
		
		// Construct Hardware
		PDP = new PowerDistributionPanel();

//INTAKE
		intakeLeftTalon = new TalonSRX(intakeLeftPort);
		intakeRightTalon = new TalonSRX(intakeRightPort);
		
		intakeLeftWrapper = new TalonSRXWrapper(ControlMode.Current, intakeLeftTalon);
		intakeRightWrapper = new TalonSRXWrapper(ControlMode.Current, intakeRightTalon);
		
//ARM
		elbowTalon = new TalonSRX(elbowPort1); 
		elbowVictor = new VictorSPX(elbowPort2); 
		wristTalon = new TalonSRX(wristPort);
		
		elbowTalon.set(ControlMode.Current, elbowPort1);
		elbowVictor.set(ControlMode.Follower, elbowPort1);
		wristTalon.set(ControlMode.Current, wristPort);
		
		elbowTalonWrapper1 = new TalonSRXWrapper(ControlMode.Current, elbowTalon);
		wristTalonWrapper = new TalonSRXWrapper(ControlMode.Current, wristTalon);
		
		elbowCurrentWrapper = new SpeedControllerWrapper(elbowTalonWrapper1);
		wristCurrentWrapper = new SpeedControllerWrapper(wristTalonWrapper);
		
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
			t.enableCurrentLimit(true);
			t.configContinuousCurrentLimit(15, 0);
			t.configPeakCurrentLimit(15, 0);
//			t.configPeakCurrentDuration(100, 0);

		}
		
		
		driveLeftVictor2.follow(driveLeftTalon);
		driveLeftVictor3.follow(driveLeftTalon);
		driveLeftVictor4.follow(driveLeftTalon);
		
		driveRightVictor2.follow(driveRightTalon);
		driveRightVictor3.follow(driveRightTalon);
		driveRightVictor4.follow(driveRightTalon);
		

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
		
//SENSORS
		pigeon = new PigeonIMU(driveRightTalon);
		irSensor = new DigitalInput(irSensorPort);
		pigeonRateWrapper = new PigeonWrapperRateAndAngle(PIDSourceType.kRate, Units.RADS);
		pigeonDisplacementWrapper = new PigeonWrapperRateAndAngle(PIDSourceType.kDisplacement, Units.RADS);

		driveLeftEncoderWrapperRate = new TalonSRXEncoderWrapper(driveLeftTalon, PIDSourceType.kRate);
		driveRightEncoderWrapperRate = new TalonSRXEncoderWrapper(driveRightTalon, PIDSourceType.kRate);
		driveLeftEncoderWrapperDistance = new TalonSRXEncoderWrapper(driveLeftTalon,  PIDSourceType.kDisplacement);
		driveRightEncoderWrapperDistance = new TalonSRXEncoderWrapper(driveRightTalon, PIDSourceType.kDisplacement);
		
		// Configure Hardware
		driveRightTalon.setSensorPhase(false);
		driveRightTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 0);
		driveRightTalon.configVelocityMeasurementWindow(1, 0);
		driveLeftTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 0);
		driveLeftTalon.configVelocityMeasurementWindow(1, 0);
		
		
		driveLeftEncoderWrapperDistance.setDistancePerRevolution(-WHEEL_RADIUS * 2 * Math.PI * 24.0 / 54);
		driveRightEncoderWrapperDistance.setDistancePerRevolution(WHEEL_RADIUS * 2 * Math.PI * 24.0 / 54);
		driveLeftEncoderWrapperRate.setDistancePerRevolution(-WHEEL_RADIUS * 2 * Math.PI * 24.0 / 54);
		driveRightEncoderWrapperRate.setDistancePerRevolution(WHEEL_RADIUS * 2 * Math.PI * 24.0 / 54);
		
		elbowEncoderWrapperDistance = new TalonSRXEncoderWrapper(elbowTalon, PIDSourceType.kDisplacement);
		wristEncoderWrapperDistance = new TalonSRXEncoderWrapper(wristTalon, PIDSourceType.kDisplacement);
		elbowEncoderWrapperRate = new TalonSRXEncoderWrapper(elbowTalon, PIDSourceType.kRate);
		wristEncoderWrapperRate = new TalonSRXEncoderWrapper(wristTalon, PIDSourceType.kRate);
		
//SUBSYSTEMS
		driveTrain = new DriveTrain();
		intake = new Intake();
		arm = new Arm();
		deadReckoning = new DeadReckoning(pigeon, driveLeftEncoderWrapperDistance, driveRightEncoderWrapperDistance);

	}

	public static void updateConstants() {
		driveTrain.updateConstants();
	}
	
}
