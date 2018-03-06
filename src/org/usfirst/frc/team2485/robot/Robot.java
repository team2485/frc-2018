
package org.usfirst.frc.team2485.robot;

import org.usfirst.frc.team2485.robot.commandGroups.ScaleAuto;
import org.usfirst.frc.team2485.robot.commandGroups.ScaleAutoCross;
import org.usfirst.frc.team2485.robot.commandGroups.SwitchAuto;
import org.usfirst.frc.team2485.robot.commands.DriveStraight;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.SetVelocities;
import org.usfirst.frc.team2485.robot.subsystems.Arm;
import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.AutoPath.Pair;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.PathTracker;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.sun.javafx.collections.MappingChange.Map;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
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
	private int i = 0;
	private boolean isHomed = true;
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
		RobotMap.init();
		ConstantsIO.init();
		OI.init();
//		RobotMap.arm.initElbowEnc();
//		RobotMap.arm.initWristEnc();
//		RobotMap.deadReckoning.start();
//		if (!RobotMap.arm.isEncodersWorking()) {
//			throw new RuntimeException("No Encoders");
//		}
		isHomed = false;

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
		RobotMap.arm.reset();
//		RobotMap.deadReckoning.stop();
		shouldCrash = false;
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		RobotMap.driveTrain.reset();
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
		if (!RobotMap.arm.isElbowCurrentSensorWorking()) {
			throw new RuntimeException("No Encoders");
		}
		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		
		//UNCOMMENT CONSTANTSIO
		RobotMap.driveRightEncoderWrapperDistance.reset();
		RobotMap.driveLeftEncoderWrapperDistance.reset();
		RobotMap.pigeon.setFusedHeading(0, 0);
		RobotMap.pigeon.setYaw(0, 0);
		RobotMap.driveLeftTalon.enableCurrentLimit(false);
		RobotMap.driveRightTalon.enableCurrentLimit(false);
		RobotMap.driveTrain.reset();

		
		if (!isHomed) {
			throw new RuntimeException("Not homed");
		}
//		
		
//		Scheduler.getInstance().add(new DriveTo(path, 90, false, 100000));
	
		
	
		
//		Scheduler.getInstance().add(new DriveStraight(170, 100, 10000));
		
		RobotMap.deadReckoning.start();
		Scheduler.getInstance().add(new SwitchAuto(true));
		

		
//		Pair[] controlPoints = {
//				new Pair(0,	0), new Pair(0, 80), new Pair(120, 80)
//			};
//			double[] dists = {
//					70
//			};
//		AutoPath path = AutoPath.getAutoPathForClothoidSpline(controlPoints, dists);
//		
//		for (int i = 0; i < 200; i++) {
//			System.out.println(path.getPointAtDist(i * path.getPathLength() / 200));
//		}
//		Scheduler.getInstance().add(new DriveTo(path, 80, false, 100000));
//			
//		RobotMap.pathTracker = new PathTracker(RobotMap.deadReckoning, path);
//		RobotMap.pathTracker.start();
//		
//		Scheduler.getInstance().add(new HighLowCurrentTest(-1, -1, -1, -1, 2000));
//		Scheduler.getInstance().add(new SetVelocities(30, 0.02));
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
		
	
		
	
//		SmartDashboard.putNumber("Drift", RobotMap.pathTracker.getDrift());
//		SmartDashboard.putNumber("Path Distance", RobotMap.pathTracker.getPathDist());
		
//		RobotMap.arm.setElbowManual(0);
//		RobotMap.arm.setWristManual(0);
//		RobotMap.arm.setElbowVelocity(.05);
//		RobotMap.arm.setElbowPos(.17);
	
		
//		RobotMap.driveTrain.setVelocities(60, 0.02);
		
		updateSmartDashboard();
//		RobotMap.drivetrain.setCurrents(-1, -1);
//		RobotMap.arm.setWristPos(.1);
//		RobotMap.arm.setWristVelocity(.05);
//		System.out.println(RobotMap.arm.wristPIDisEnabled());
		if (shouldCrash) {
			throw new RuntimeException("Current Sensor failed");
		}
	}

	@Override
	public void teleopInit() {
		ConstantsIO.init();
		RobotMap.updateConstants();
		RobotMap.driveTrain.reset();
		RobotMap.arm.reset();
		if (!isHomed) {
			throw new RuntimeException("Not homed");
		}
		
		if (!startedCamera) {
//			UsbCamera jevoisCam = CameraServer.getInstance().startAutomaticCapture();
//			jevoisCam.setVideoMode(PixelFormat.kYUYV, 160, 120, 30);
			startedCamera = true;
		}
//		
//		if (!RobotMap.arm.isEncodersWorking()) {
//			throw new RuntimeException("No Encoders");
//		}
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
		
		if (!RobotMap.arm.isElbowCurrentSensorWorking()) {
			i++;
			if (i > 20) {
//				throw new RuntimeException("No Encoders");
			}
		} else {
			i = 0;
		}
		
		if (shouldCrash) {
			throw new RuntimeException("Current Sensor failed");
		}
//		RobotMap.intakeLeftTalon.set(ControlMode.PercentOutput, ThresholdHandler.deadbandAndScale(OI.operator.getRawAxis(OI.XBOX_LYJOYSTICK_PORT), .2, 0, 1));
//		RobotMap.intakeRightTalon.set(ControlMode.PercentOutput, ThresholdHandler.deadbandAndScale(OI.operator.getRawAxis(OI.XBOX_RYJOYSTICK_PORT), .2, 0, 1));
//		RobotMap.arm.setWristCurrent(1);
//		RobotMap.arm.setElbowCurrent(1);
	
//		RobotMap.arm.setWristManual(ThresholdHandler.deadbandAndScale(OI.driver.getRawAxis(1), OI.XBOX_AXIS_DEADBAND, 0, 1));
//		RobotMap.arm.setElbowManual(ThresholdHandler.deadbandAndScale(OI.driver.getRawAxis(5), OI.XBOX_AXIS_DEADBAND, 0, 1));
//		RobotMap.elbowTalon.set(ControlMode.PercentOutput, -ThresholdHandler.deadbandAndScale(OI.driver.getRawAxis(5), OI.XBOX_AXIS_DEADBAND, 0, 1));
//		RobotMap.wristTalon.set(ControlMode.PercentOutput, -ThresholdHandler.deadbandAndScale(OI.driver.getRawAxis(1), OI.XBOX_AXIS_DEADBAND, 0, 1));

	}
	

	public void updateSmartDashboard() {
	
		SmartDashboard.putNumber("Yaw", RobotMap.pigeonDisplacementWrapper.pidGet());
		SmartDashboard.putNumber("Yaw Rate", RobotMap.pigeonRateWrapper.pidGet());
		SmartDashboard.putNumber("velocity setpoint", RobotMap.driveTrain.velocitySetpointTN.getOutput());
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
		SmartDashboard.putNumber("Relative Wrist", RobotMap.wristTalon.getSensorCollection().getQuadraturePosition());
		SmartDashboard.putNumber("Relative Elbow", RobotMap.elbowTalon.getSensorCollection().getQuadraturePosition());
		
		SmartDashboard.putNumber("Left Current Error", RobotMap.driveLeftTalon.getClosedLoopError(0));
		SmartDashboard.putNumber("Right Current Error", -RobotMap.driveRightTalon.getClosedLoopError(0));
		
		SmartDashboard.putNumber("Velocity TN", RobotMap.driveTrain.velocityTN.pidGet());
		SmartDashboard.putNumber("Ang Vel TN", RobotMap.driveTrain.curvatureTN.pidGet());
		SmartDashboard.putNumber("Current", (Math.abs(RobotMap.driveLeftTalon.getOutputCurrent()) + Math.abs(RobotMap.driveLeftTalon.getOutputCurrent()))/2);


		SmartDashboard.putNumber("Curvature Setpoint TN", RobotMap.driveTrain.curvatureSetpointTN.getOutput());
		SmartDashboard.putNumber("Elbow Encoder", RobotMap.elbowEncoderWrapperDistance.pidGet());
		SmartDashboard.putNumber("Wrist Encoder", RobotMap.wristEncoderWrapperDistance.pidGet());
		SmartDashboard.putNumber("Intake Current", RobotMap.intakeLeftTalon.getOutputCurrent());
		SmartDashboard.putNumber("X", RobotMap.deadReckoning.getX());
		SmartDashboard.putNumber("Y", RobotMap.deadReckoning.getY());
		SmartDashboard.putNumber("Elbow Current error", RobotMap.elbowTalon.getClosedLoopError(0));
		SmartDashboard.putNumber("Wrist Current Error", RobotMap.wristTalon.getClosedLoopError(0));
		SmartDashboard.putNumber("Wrist Current", RobotMap.wristTalon.getOutputCurrent());
		SmartDashboard.putNumber("Wrist Ang Vel error", RobotMap.arm.getWristAngVelError());
		SmartDashboard.putNumber("Wrist Enc Rate", RobotMap.wristEncoderWrapperRate.pidGet());
		SmartDashboard.putNumber("Elbow Enc Rate", RobotMap.elbowEncoderWrapperRate.pidGet());
		SmartDashboard.putNumber("Wrist Ang Error", RobotMap.arm.getWristAngError());
		SmartDashboard.putNumber("Elbow Ang Vel Error", RobotMap.arm.getElbowAngVelError());
		SmartDashboard.putNumber("Elbow Ang Error", RobotMap.arm.getElbowAngError());
		SmartDashboard.putNumber("Elbow Enc Raw", RobotMap.elbowTalon.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Wrist Current", RobotMap.wristTalon.getOutputCurrent());
		SmartDashboard.putNumber("Elbow Current", RobotMap.elbowTalon.getOutputCurrent());
		SmartDashboard.putNumber("Wrist Ang TN", RobotMap.arm.wristAngTN.getOutput());
		SmartDashboard.putNumber("Wrist Max Current Source", RobotMap.arm.wristMaxCurrentSource.pidGet());
		
		SmartDashboard.putBoolean("IR Sensor", RobotMap.irSensor.get());
		
		SmartDashboard.putNumber("Max Velocity OR Source", RobotMap.driveTrain.maxVelocityORSource.pidGet());
		
		SmartDashboard.putNumber("Max Ang Vel Elbow OR Source", RobotMap.arm.elbowMaxAngVelSource.pidGet());
		SmartDashboard.putNumber("Min Ang Vel Elbow OR Source", RobotMap.arm.elbowMinAngVelSource.pidGet());
		
		


		
		SmartDashboard.putNumber("Wrist PercentOutput", RobotMap.wristTalon.getMotorOutputPercent());
		SmartDashboard.putNumber("Elbow PercentOutput", RobotMap.elbowTalon.getMotorOutputPercent());
		
		SmartDashboard.putNumber("Angle Setpoint TN", RobotMap.driveTrain.angleSetpointTN.getOutput());
		

		
//		SmartDashboard.putNumber("Wrist Encoder Distance", RobotMap.wristEncoderWrapperDistance.pidGet());
//		RobotMap.pigeon.getYawPitchRoll(ypr);
		
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
		} else {
			RobotMap.arm.setWristCurrent(0);
			RobotMap.arm.setElbowCurrent(0);
		}
		RobotMap.elbowEncoderWrapperDistance.setPosition(-.198);
		RobotMap.wristEncoderWrapperDistance.setPosition(0.421);
		isHomed = true;
	}
	

}
