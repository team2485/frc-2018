package org.usfirst.frc.team2485.robot;

import com.kauailabs.navx.frc.AHRS;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	public static double ROBOT_WIDTH;
	
	public static AHRS ahrs;
	
	public static void init() {
		
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
