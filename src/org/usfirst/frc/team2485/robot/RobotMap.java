package org.usfirst.frc.team2485.robot;


import com.ctre.phoenix.sensors.PigeonIMU;

import org.usfirst.frc.team2485.robot.subsystems.DriveTrain;


import com.kauailabs.navx.frc.AHRS;

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
	
	public static void init() {
		pigeon = new PigeonIMU(PIGEON_PORT);


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
