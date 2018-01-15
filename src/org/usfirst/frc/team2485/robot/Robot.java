
package org.usfirst.frc.team2485.robot;

import org.usfirst.frc.team2485.robot.commands.DriveStraight;
import org.usfirst.frc.team2485.robot.commands.HighLowCurrentTest;
import org.usfirst.frc.team2485.robot.commands.SetVelocities;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		OI.init();
		RobotMap.init();
		ConstantsIO.init();
		RobotMap.updateConstants();
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		Scheduler.getInstance().removeAll();
		RobotMap.drivetrain.reset();
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		updateSmartDashboard();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		ConstantsIO.init();
		RobotMap.updateConstants();
		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		RobotMap.driveRightEncoderWrapperDistance.reset();
		RobotMap.driveLeftEncoderWrapperDistance.reset();
		RobotMap.pigeon.setFusedHeading(0, 0);
//		Scheduler.getInstance().add(new HighLowCurrentTest(-8, -4, -8, -4, 4000));
//		Scheduler.getInstance().add(new SetVelocities(30, 0));
		Scheduler.getInstance().add(new DriveStraight(1200, 100, 100000));


	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
//		RobotMap.drivetrain.driveTo(200, 100, 0, 0);
		updateSmartDashboard();
//		RobotMap.drivetrain.setCurrents(-1, -1);
	}

	@Override
	public void teleopInit() {
		ConstantsIO.init();
		RobotMap.updateConstants();
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		
		
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		updateSmartDashboard();
		double y = -ThresholdHandler.deadbandAndScale(OI.XBOX.getRawAxis(OI.XBOX_LYJOYSTICK_PORT), RobotMap.drivetrain.THROTTLE_DEADBAND, 0, 1);
    	double x = ThresholdHandler.deadbandAndScale(OI.XBOX.getRawAxis(OI.XBOX_RXJOYSTICK_PORT), RobotMap.drivetrain.STEERING_DEADBAND, 0, 1);;
    	
    	RobotMap.drivetrain.simpleDrive(y, x);

	}

	public void updateSmartDashboard() {
		SmartDashboard.putNumber("Yaw", RobotMap.pigeonDisplacementWrapper.pidGet());
		SmartDashboard.putNumber("Yaw Rate", RobotMap.pigeonRateWrapper.pidGet());
		SmartDashboard.putNumber("Yaw Rate Error", RobotMap.drivetrain.getAngleRateError());
		SmartDashboard.putNumber("Left Encoder Dist", RobotMap.driveLeftEncoderWrapperDistance.pidGet());
		SmartDashboard.putNumber("Right Encoder Dist", RobotMap.driveRightEncoderWrapperDistance.pidGet());
		SmartDashboard.putNumber("Left Encoder Rate", RobotMap.driveLeftEncoderWrapperRate.pidGet());
		SmartDashboard.putNumber("Right Encoder Rate", RobotMap.driveRightEncoderWrapperRate.pidGet());
		SmartDashboard.putNumber("Velocity Error", RobotMap.drivetrain.getVelocityError());
		SmartDashboard.putNumber("Distance Error", RobotMap.drivetrain.getDistError());
		SmartDashboard.putNumber("Angle Error", RobotMap.drivetrain.getAngleError());
		
		SmartDashboard.putNumber("Left Current", RobotMap.driveLeftTalon1.getOutputCurrent());
		SmartDashboard.putNumber("Right Current", RobotMap.driveRightTalon1.getOutputCurrent());

		
		SmartDashboard.putNumber("Left Current Error", RobotMap.driveLeftTalon1.getClosedLoopError(0));
		SmartDashboard.putNumber("Right Current Error", RobotMap.driveRightTalon1.getClosedLoopError(0));
		
		SmartDashboard.putNumber("Velocity TN", RobotMap.drivetrain.velocityTN.pidGet());
		SmartDashboard.putNumber("Ang Vel TN", RobotMap.drivetrain.angVelocityTN.pidGet());
		SmartDashboard.putNumber("Left + Right", RobotMap.driveLeftTalonCurrentWrapper1.get() + RobotMap.driveRightTalonCurrentWrapper1.get());
	}
	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}

}
