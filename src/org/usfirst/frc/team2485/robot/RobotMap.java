package org.usfirst.frc.team2485.robot;

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
	
	//SENSORS
	public static AHRS ahrs;
	
	//SPEED CONTROLLERS
	
	//SUBSYSTEMS
	public static DriveTrain drivetrain;
	
	public static void init() {
		
		//SENSORS
		ahrs = new AHRS(Port.kMXP);
		
		//SUBSYSTEMS
		drivetrain = new DriveTrain();
		
		
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
