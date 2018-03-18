
package org.usfirst.frc.team2485.robot;

import org.usfirst.frc.team2485.robot.commandGroups.ScaleAuto;
import org.usfirst.frc.team2485.robot.commandGroups.SwitchAuto;
import org.usfirst.frc.team2485.robot.commands.AngleVelocityTest;
import org.usfirst.frc.team2485.robot.commands.DriveStraight;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.HighLowCurrentTest;
import org.usfirst.frc.team2485.robot.commands.SetVelocities;
import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.FastMath;
import org.usfirst.frc.team2485.util.AutoPath.Pair;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	private boolean isHomed = false;
	private static boolean shouldCrash = false;
	private boolean startedCamera = false;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	
	public static void forceDisable() {
		shouldCrash = true;
	}
	
	@Override
	public void robotInit() {
		FastMath.init();

		RobotMap.init();
		ConstantsIO.init();
		OI.init();
		RobotMap.updateConstants();
		

		ScaleAuto.init();
		SwitchAuto.init();
		
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
		RobotMap.arm.reset();
		shouldCrash = false;
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		RobotMap.driveTrain.reset();
//		updateSmartDashboard();
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
//		RobotMap.elbowEncoderWrapperDistance.setPosition(-.190);
//		RobotMap.wristEncoderWrapperDistance.setPosition(0.416);
		
//		isHomed = true; // so we don't crash immediately in actual matches		

		String positions = DriverStation.getInstance().getGameSpecificMessage().toUpperCase();
		boolean switchLeft = positions.charAt(0) == 'L';
		boolean scaleLeft = positions.charAt(1) == 'L';
		
		RobotMap.driveRightEncoderWrapperDistance.reset();
		RobotMap.driveLeftEncoderWrapperDistance.reset();
		RobotMap.pigeon.setFusedHeading(0, 0);
		RobotMap.pigeon.setYaw(0, 0);
		RobotMap.driveLeftTalon.enableCurrentLimit(false);
		RobotMap.driveRightTalon.enableCurrentLimit(false);
		RobotMap.driveTrain.reset();
				
		// CHANGE AUTO HERE
		boolean startLeft = false;
//		Scheduler.getInstance().add(new SetVelocities(30, 0));
//		Scheduler.getInstance().add(new HighLowCurrentTest(5, 2, 5, 2, 4000));
//		Scheduler.getInstance().add(new ScaleAuto(startLeft, scaleLeft));

		
		//Current PID testing
//		Scheduler.getInstance().add(new HighLowCurrentTest(10, 5, 10, 5, 100)); 
		
		//Velocity PID testing
//		Scheduler.getInstance().add(new DriveStraight(100, 100, //needs to be set!1000)); 
		
		//Distance PID testing
//		Scheduler.getInstance().add(new DriveStraight(200, 30, 1000));
//		Scheduler.getInstance().add(new DriveStraight(300, 100, 1000));

		//Angle Velocity PID testing
//		Scheduler.getInstance().add(new AngleVelocityTest(.3, -.3, 7270)); //please check this

		//Angle PID testing
//		AutoPath path = new AutoPath(AutoPath.getPointsForBezier(
//		));
//		Scheduler.getInstance().add(new DriveTo(path, 60, false, 8000, false));
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
//		RobotMap.arm.wristAngPID.enable();
//		RobotMap.arm.elbowAngVelMaxPID.enable();
//		RobotMap.arm.elbowAngVelMinPID.enable();
//		RobotMap.arm.elbowAngPID.enable();
//		RobotMap.arm.elbowAngPID.setSetpoint(0);
//		RobotMap.arm.wristAngPID.setSetpoint(.2);
		
		updateSmartDashboard();
//		if (shouldCrash) {
//			throw new RuntimeException("Current Sensor failed");
//		}
	}

	@Override
	public void teleopInit() {
		ConstantsIO.init();
		RobotMap.updateConstants();
		RobotMap.driveTrain.reset();
		RobotMap.arm.reset();
		if (!isHomed) { // set to true in auto init, only relevant for pit testing
			throw new RuntimeException("Not homed");
		}
		
		if (!startedCamera) {
			UsbCamera jevoisCam = CameraServer.getInstance().startAutomaticCapture();
			jevoisCam.setVideoMode(PixelFormat.kYUYV, 160, 120, 30);
			startedCamera = true;
		}
		
		
		
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		updateSmartDashboard();
		
		
//		if (shouldCrash) {
//			throw new RuntimeException("Current Sensor failed");
//		}
	}
	

	public void updateSmartDashboard() {
	
//		SmartDashboard.putNumber("Yaw", RobotMap.pigeonDisplacementWrapper.pidGet());
//		SmartDashboard.putNumber("Yaw Rate", RobotMap.pigeonRateWrapper.pidGet());
//		SmartDashboard.putNumber("velocity setpoint", RobotMap.driveTrain.velocitySetpointTN.getOutput());
//		SmartDashboard.putNumber("Yaw Rate Error", RobotMap.driveTrain.getAngleRateError());
		SmartDashboard.putNumber("Left Encoder Dist", RobotMap.driveLeftEncoderWrapperDistance.pidGet());
		SmartDashboard.putNumber("Right Encoder Dist", RobotMap.driveRightEncoderWrapperDistance.pidGet());
//		SmartDashboard.putNumber("Left Encoder Rate", RobotMap.driveLeftEncoderWrapperRate.pidGet());
//		SmartDashboard.putNumber("Right Encoder Rate", RobotMap.driveRightEncoderWrapperRate.pidGet());
		SmartDashboard.putNumber("Velocity Error", RobotMap.driveTrain.getVelocityError());
		SmartDashboard.putNumber("Distance Error", RobotMap.driveTrain.getDistError());
		SmartDashboard.putNumber("Angle Error", RobotMap.driveTrain.getAngleError());
		SmartDashboard.putNumber("Angular Velocity Error", RobotMap.driveTrain.getAngleRateError());
//	
//		SmartDashboard.putNumber("Distance PID Output", RobotMap.driveTrain.getDistancePIDOutput());
//		SmartDashboard.putNumber("Velocity PID Output", RobotMap.driveTrain.getVelocityPIDOutput());
//		SmartDashboard.putNumber("Left Current Output", RobotMap.driveLeftTalon.getMotorOutputPercent());
		SmartDashboard.putNumber("Left Current", RobotMap.driveLeftTalon.getOutputCurrent());
		SmartDashboard.putNumber("Right Current", RobotMap.driveRightTalon.getOutputCurrent());
//		SmartDashboard.putNumber("Relative Wrist", RobotMap.wristTalon.getSensorCollection().getQuadraturePosition());
//		SmartDashboard.putNumber("Relative Elbow", RobotMap.elbowTalon.getSensorCollection().getQuadraturePosition());
//		
		SmartDashboard.putNumber("Left Current Error", RobotMap.driveLeftTalon.getClosedLoopError(0));
		SmartDashboard.putNumber("Right Current Error", -RobotMap.driveRightTalon.getClosedLoopError(0));
//		
//		SmartDashboard.putNumber("Velocity TN", RobotMap.driveTrain.velocityTN.pidGet());
//		SmartDashboard.putNumber("Ang Vel TN", RobotMap.driveTrain.angularVelocityTN.pidGet());
//		SmartDashboard.putNumber("Current", (Math.abs(RobotMap.driveLeftTalon.getOutputCurrent()) + Math.abs(RobotMap.driveLeftTalon.getOutputCurrent()))/2);
//
//
//		SmartDashboard.putNumber("Curvature Setpoint TN", RobotMap.driveTrain.curvatureSetpointTN.getOutput());
		SmartDashboard.putNumber("Elbow Encoder", RobotMap.elbowEncoderWrapperDistance.pidGet());
		SmartDashboard.putNumber("Wrist Encoder", RobotMap.wristEncoderWrapperDistance.pidGet());
//		SmartDashboard.putNumber("Intake Current", RobotMap.intakeLeftTalon.getOutputCurrent());
//		SmartDashboard.putNumber("X", RobotMap.deadReckoning.getX());
//		SmartDashboard.putNumber("Y", RobotMap.deadReckoning.getY());
//		SmartDashboard.putNumber("Elbow Current error", RobotMap.elbowTalon.getClosedLoopError(0));
//		SmartDashboard.putNumber("Wrist Current Error", RobotMap.wristTalon.getClosedLoopError(0));
//		SmartDashboard.putNumber("Wrist Current", RobotMap.wristTalon.getOutputCurrent());
//		SmartDashboard.putNumber("Wrist Enc Rate", RobotMap.wristEncoderWrapperRate.pidGet());
//		SmartDashboard.putNumber("Elbow Enc Rate", RobotMap.elbowEncoderWrapperRate.pidGet());
		SmartDashboard.putNumber("Wrist Ang Error", RobotMap.arm.getWristAngError());
		SmartDashboard.putNumber("Elbow Ang Vel Max Error", RobotMap.arm.getElbowAngVelMaxError());
		SmartDashboard.putNumber("Elbow Ang Vel Min Error", RobotMap.arm.getElbowAngVelMinError());
		SmartDashboard.putNumber("Elbow Ang Error", RobotMap.arm.getElbowAngError());
//		SmartDashboard.putNumber("Elbow Enc Raw", RobotMap.elbowTalon.getSelectedSensorPosition(0));
//		SmartDashboard.putNumber("Wrist Current", RobotMap.wristTalon.getOutputCurrent());
		SmartDashboard.putNumber("Elbow Current", RobotMap.elbowTalon.getOutputCurrent());
		SmartDashboard.putNumber("Drive Ang Vel Setpoint", RobotMap.driveTrain.angularVelocityPID.getSetpoint());
		
		SmartDashboard.putNumber("Wrist Min Current", RobotMap.arm.wristMinCurrentSource.pidGet());
		SmartDashboard.putNumber("Wrist Max Current", RobotMap.arm.wristMaxCurrentSource.pidGet());
		SmartDashboard.putNumber("Wrist Current", RobotMap.wristTalon.getOutputCurrent());
//		SmartDashboard.putNumber("Wrist Ang TN", RobotMap.arm.wristAngTN.getOutput());
//		SmartDashboard.putNumber("Wrist Max Current Source", RobotMap.arm.wristMaxCurrentSource.pidGet());
//		
//		SmartDashboard.putBoolean("IR Sensor", RobotMap.irSensor.get());
//		
//		SmartDashboard.putNumber("Max Velocity OR Source", RobotMap.driveTrain.maxVelocityORSource.pidGet());
//		
//		SmartDashboard.putNumber("Max Ang Vel Elbow OR Source", RobotMap.arm.elbowMaxAngVelSource.pidGet());
//		SmartDashboard.putNumber("Min Ang Vel Elbow OR Source", RobotMap.arm.elbowMinAngVelSource.pidGet());
//		
//		
//		SmartDashboard.putNumber("Drive Straight Error", RobotMap.driveTrain.driveStraightPID.getAvgError());
//
//		SmartDashboard.putNumber("Average Speed", RobotMap.driveTrain.getAverageSpeed());
//		
//		SmartDashboard.putNumber("Wrist PercentOutput", RobotMap.wristTalon.getMotorOutputPercent());
//		SmartDashboard.putNumber("Elbow PercentOutput", RobotMap.elbowTalon.getMotorOutputPercent());
//		
//		SmartDashboard.putNumber("Angle Setpoint TN", RobotMap.driveTrain.angleSetpointTN.getOutput());
//		
//		SmartDashboard.putNumber("Drift", RobotMap.pathTracker.getDrift());
//		
//		SmartDashboard.putNumber("Left Percent Output", RobotMap.driveLeftTalon.getMotorOutputPercent());
//		SmartDashboard.putNumber("Right Percent Output", RobotMap.driveRightTalon.getMotorOutputPercent());
//		SmartDashboard.putNumber("Left Drive Error", RobotMap.driveLeftTalon.getClosedLoopError(0));
//		SmartDashboard.putNumber("Right Drive Error", RobotMap.driveRightTalon.getClosedLoopError(0));
//		
//
//		
//		SmartDashboard.putNumber("Wrist Encoder Distance", RobotMap.wristEncoderWrapperDistance.pidGet());
//		
//		SmartDashboard.putNumber("X", RobotMap.deadReckoning.getX());
//		SmartDashboard.putNumber("Y", RobotMap.deadReckoning.getY());
	}
	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		if (OI.driver.getRawButton(OI.XBOX_A_PORT)) {
			RobotMap.arm.setElbowCurrent(-2);
			RobotMap.arm.setWristCurrent(0);
		} else if (OI.driver.getRawButton(OI.XBOX_B_PORT)) {
			RobotMap.arm.setWristCurrent(3);
			RobotMap.arm.setElbowCurrent(0);
		} else if (OI.driver.getRawButton(OI.XBOX_Y_PORT)) {
			RobotMap.arm.setWristCurrent(-2);
			RobotMap.arm.setElbowCurrent(0);
		} else if (OI.driver.getRawButton(OI.XBOX_X_PORT)) {
			RobotMap.arm.setElbowCurrent(2);
			RobotMap.arm.setWristCurrent(0);
		} else {
			RobotMap.arm.setWristCurrent(0);
			RobotMap.arm.setElbowCurrent(0);
		}
		RobotMap.elbowEncoderWrapperDistance.setPosition(-.190);
		RobotMap.wristEncoderWrapperDistance.setPosition(0.416);
		isHomed = true;
	}
	

}
