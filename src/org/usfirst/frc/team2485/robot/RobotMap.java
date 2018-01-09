package org.usfirst.frc.team2485.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import java.sql.Wrapper;

import org.usfirst.frc.team2485.robot.subsystems.DriveTrain;
import org.usfirst.frc.team2485.util.EncoderWrapperRateAndDistance;
import org.usfirst.frc.team2485.util.TalonSRXEncoderWrapper;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SerialPort.Port;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	public static double ROBOT_WIDTH;

	public static final int PIGEON_PORT = 0;

	public static AHRS ahrs;

	public static PigeonIMU pigeon;

	public static TalonSRX leftDriveTalon1;
	public static TalonSRX leftDriveTalon2;
	public static TalonSRX leftDriveTalon3;
	public static TalonSRX rightDriveTalon1;
	public static TalonSRX rightDriveTalon2;
	public static TalonSRX rightDriveTalon3;
	
	public static int driveRightPort1 = 1;
	public static int driveRightPort2 = 3;
	public static int driveRightPort3 = 2;
	public static int driveLeftPort1 = 4;
	public static int driveLeftPort2 = 6;
	public static int driveLeftPort3 = 5;

	public static TalonSRXEncoderWrapper leftDriveEncoderWrapperRate;
	public static TalonSRXEncoderWrapper rightDriveEncoderWrapperRate;
	public static TalonSRXEncoderWrapper leftDriveEncoderWrapperDistance;
	public static TalonSRXEncoderWrapper rightDriveEncoderWrapperDistance;

	public static void init() {
		pigeon = new PigeonIMU(PIGEON_PORT);

		leftDriveTalon1 = new TalonSRX(driveLeftPort1);

		leftDriveTalon2 = new TalonSRX(driveLeftPort2);

		leftDriveTalon3 = new TalonSRX(driveLeftPort3);

		rightDriveTalon1 = new TalonSRX(driveRightPort1);

		rightDriveTalon2 = new TalonSRX(driveRightPort2);

		rightDriveTalon3 = new TalonSRX(driveRightPort3);

		leftDriveEncoderWrapperRate = new TalonSRXEncoderWrapper(leftDriveTalon3, PIDSourceType.kRate);

		rightDriveEncoderWrapperRate = new TalonSRXEncoderWrapper(rightDriveTalon3, PIDSourceType.kRate);
		
		leftDriveEncoderWrapperDistance = new TalonSRXEncoderWrapper(leftDriveTalon3,  PIDSourceType.kDisplacement);
		
		rightDriveEncoderWrapperDistance = new TalonSRXEncoderWrapper(rightDriveTalon3, PIDSourceType.kDisplacement);
	}

	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
}
