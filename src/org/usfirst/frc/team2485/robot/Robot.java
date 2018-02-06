
package org.usfirst.frc.team2485.robot;

import org.usfirst.frc.team2485.robot.commands.DriveStraight;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.HighLowCurrentTest;
import org.usfirst.frc.team2485.robot.commands.ResetDriveTrain;
import org.usfirst.frc.team2485.robot.commands.SetVelocities;
import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.DeadReckoning;
import org.usfirst.frc.team2485.util.ThresholdHandler;
import org.usfirst.frc.team2485.util.AutoPath.Pair;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
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
	double[] ypr = new double[3];

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		OI.init();
		RobotMap.init();
//		ConstantsIO.init();
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
		RobotMap.driveTrain.reset();
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
//		ConstantsIO.init();
		RobotMap.updateConstants();
		RobotMap.driveTrain.reset();
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
		Scheduler.getInstance().add(new HighLowCurrentTest(-1, -1, -1, -1, 2000));
//		Scheduler.getInstance().add(new SetVelocities(60, 0));
//		CommandGroup cg = new CommandGroup();
//		cg.addSequential(new DriveTo(new AutoPath(AutoPath.getPointsForBezier(10000, new Pair(0, 0), 
//				new Pair(0, 100), new Pair(100, 100))), 100, false, 5000));
//		cg.addSequential(new ResetDriveTrain());
//		Scheduler.getInstance().add(cg);


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
//		ConstantsIO.init();
		RobotMap.updateConstants();
		RobotMap.driveTrain.reset();
//		RobotMap.deadReckoning.start();
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


	}

	public void updateSmartDashboard() {

//		SmartDashboard.putNumber("Yaw", RobotMap.pigeonDisplacementWrapper.pidGet());
//		SmartDashboard.putNumber("Yaw Rate", RobotMap.pigeonRateWrapper.pidGet());
		SmartDashboard.putNumber("Yaw Rate Error", RobotMap.driveTrain.getAngleRateError());
		SmartDashboard.putNumber("Left Encoder Dist", RobotMap.driveLeftEncoderWrapperDistance.pidGet());
		SmartDashboard.putNumber("Right Encoder Dist", RobotMap.driveRightEncoderWrapperDistance.pidGet());
		SmartDashboard.putNumber("Left Encoder Rate", RobotMap.driveLeftEncoderWrapperRate.pidGet());
		SmartDashboard.putNumber("Right Encoder Rate", RobotMap.driveRightEncoderWrapperRate.pidGet());
		SmartDashboard.putNumber("Velocity Error", RobotMap.driveTrain.getVelocityError());
		SmartDashboard.putNumber("Distance Error", RobotMap.driveTrain.getDistError());
		SmartDashboard.putNumber("Angle Error", RobotMap.driveTrain.getAngleError());
		SmartDashboard.putNumber("Angular Velocity Error", RobotMap.driveTrain.getAngleRateError());
	
		SmartDashboard.putNumber("Distance PID Output", RobotMap.driveTrain.getDistancePIDOutput());
		SmartDashboard.putNumber("Velocity PID Output", RobotMap.driveTrain.getVelocityPIDOutput());
		SmartDashboard.putNumber("Left Current Output", RobotMap.driveLeftTalon.getMotorOutputPercent());
		SmartDashboard.putNumber("Left Current", RobotMap.driveLeftTalon.getOutputCurrent());
		SmartDashboard.putNumber("Right Current", RobotMap.driveRightTalon.getOutputCurrent());

		
		SmartDashboard.putNumber("Left Current Error", RobotMap.driveLeftTalon.getClosedLoopError(0));
		SmartDashboard.putNumber("Right Current Error", -RobotMap.driveRightTalon.getClosedLoopError(0));
		
		SmartDashboard.putNumber("Velocity TN", RobotMap.driveTrain.velocityTN.pidGet());
		SmartDashboard.putNumber("Ang Vel TN", RobotMap.driveTrain.curvatureTN.pidGet());
		SmartDashboard.putNumber("Current", (Math.abs(RobotMap.driveLeftTalon.getOutputCurrent()) + Math.abs(RobotMap.driveLeftTalon.getOutputCurrent()))/2);
		SmartDashboard.putNumber("Curvature Error", RobotMap.driveTrain.getTeleopAngVelError());
		SmartDashboard.putNumber("Curvature Error", RobotMap.driveTrain.getTeleopAngVelError());
//		RobotMap.pigeon.getYawPitchRoll(ypr);
		
//		SmartDashboard.putNumber("X", RobotMap.deadReckoning.getX());
//		SmartDashboard.putNumber("Y", RobotMap.deadReckoning.getY());
	}
	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}

}
