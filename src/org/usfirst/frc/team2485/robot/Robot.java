
package org.usfirst.frc.team2485.robot;

import java.util.ArrayList;

import org.usfirst.frc.team2485.robot.commandGroups.ChampsCrossAuto;
import org.usfirst.frc.team2485.robot.commandGroups.Eject;
import org.usfirst.frc.team2485.robot.commandGroups.ScaleAuto;
import org.usfirst.frc.team2485.robot.commandGroups.SwitchAuto;
import org.usfirst.frc.team2485.robot.commands.ArmSetSetpoint;
import org.usfirst.frc.team2485.robot.commands.CancelCommand;
import org.usfirst.frc.team2485.robot.commands.ResetDriveTrain;
import org.usfirst.frc.team2485.robot.commands.RotateTo;
import org.usfirst.frc.team2485.robot.subsystems.Arm.ArmSetpoint;
import org.usfirst.frc.team2485.util.AutoLogger;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.Event.Type;
import org.usfirst.frc.team2485.util.FastMath;
import org.usfirst.frc.team2485.util.ThresholdHandler;


import com.ctre.phoenix.Logger;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import javafx.scene.transform.Rotate;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

public class Robot extends IterativeRobot {

	private boolean isHomed = false;
	private boolean startedCamera = false;
	public static boolean operatorBackup = false, driverBackup = false;
	boolean debug;
	private Command auto; 

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	
	
	@Override
	public void robotInit() {
		FastMath.init();

		RobotMap.init();
		ConstantsIO.init();
		OI.init();
		RobotMap.updateConstants();
		

		ScaleAuto.init();
		SwitchAuto.init();
		
		debug = false;
		
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		Scheduler.getInstance().removeAll();
		if (debug) {
			RobotMap.driveTrain.reset();
			RobotMap.arm.reset();
			RobotMap.intake.setRollers(0);
		}
		AutoLogger.write();
	}
	int counter = 0;
	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		RobotMap.driveTrain.reset();
		updateSmartDashboard();
		if (counter++ % 50 == 0) {
			byte b = (byte)( counter / 50) ;
			byte[] arr = new byte[100];
			System.out.println(RobotMap.leds.transaction(new byte[] {b}, arr, 1));
			System.out.println(arr[0]);
		}
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
	@SuppressWarnings("unused")
	public void autonomousInit() {
		ConstantsIO.init();
		RobotMap.updateConstants();
		
		RobotMap.elbowEncoderWrapperDistance.setPosition(-.190);
		RobotMap.wristEncoderWrapperDistance.setPosition(0.416);
		isHomed = true; // so we don't crash immediately in actual matches	
		
//		if (!isHomed) {
//			throw new RuntimeException("Not homed");
//		}
//		
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
		RobotMap.deadReckoning.start();
		
		//Testing
//		CommandGroup drive = new CommandGroup();
//		CommandGroup arm = new CommandGroup();
//		drive.addSequential(new RotateTo(Math.PI/2, 10000));
//		drive.addSequential(new ResetDriveTrain());
//		arm.addSequential(new ArmSetSetpoint(ArmSetpoint.SCALE_LOW_BACK));
//		CommandGroup otherThings = new CommandGroup();
//		otherThings.addParallel(drive);
//		otherThings.addParallel(arm);
//		CommandGroup everything = new CommandGroup();
//		everything.addSequential(otherThings);
//		everything.addSequential(new Eject(false, true));
//		Scheduler.getInstance().add(everything);

				
		// CHANGE AUTO HERE
		boolean startLeft = false;
//		auto = new SwitchAuto(switchLeft, scaleLeft);
		auto = new ScaleAuto(startLeft, switchLeft, scaleLeft);
//		auto = new ScaleAuto(startLeft, switchLeft, scaleLeft);
		Scheduler.getInstance().add(auto);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
//		RobotMap.pathTracker.start(ScaleAuto.pathLeftCross);
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
		if (debug) {
			RobotMap.driveTrain.reset();
			RobotMap.arm.reset();
		}
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
		
		boolean newButtonHeld = OI.operator.getRawButton(OI.XBOX_LBUMPER_PORT) ||
    			OI.operator.getRawButton(OI.XBOX_RBUMPER_PORT) ||
    			OI.operator.getRawButton(OI.XBOX_A_PORT) ||
    			OI.operator.getRawButton(OI.XBOX_B_PORT) ||
    			OI.operator.getRawButton(OI.XBOX_X_PORT) ||
    			OI.operator.getRawButton(OI.XBOX_Y_PORT) || 
    			OI.operator.getRawButton(OI.XBOX_BACK_BUTTON) ||
    			OI.operator.getRawButton(OI.XBOX_START_BUTTON) || OI.operatorBackup.getRawButton(OI.XBOX_LBUMPER_PORT) ||
    			OI.operatorBackup.getRawButton(OI.XBOX_RBUMPER_PORT) ||
    			OI.operatorBackup.getRawButton(OI.XBOX_A_PORT) ||
    			OI.operatorBackup.getRawButton(OI.XBOX_B_PORT) ||
    			OI.operatorBackup.getRawButton(OI.XBOX_X_PORT) ||
    			OI.operatorBackup.getRawButton(OI.XBOX_Y_PORT) || 
    			OI.operatorBackup.getRawButton(OI.XBOX_BACK_BUTTON) ||
    			OI.operatorBackup.getRawButton(OI.XBOX_START_BUTTON);
		
		if ((OI.driver.getRawAxis(OI.XBOX_RTRIGGER_PORT) > OI.XBOX_TRIGGER_DEADBAND || 
				OI.driver.getRawAxis(OI.XBOX_LTRIGGER_PORT) > OI.XBOX_TRIGGER_DEADBAND || 
				OI.driver.getRawButton(OI.XBOX_X_PORT) || OI.driverBackup.getRawAxis(OI.XBOX_RTRIGGER_PORT) > OI.XBOX_TRIGGER_DEADBAND 
				|| OI.driverBackup.getRawAxis(OI.XBOX_LTRIGGER_PORT) > OI.XBOX_TRIGGER_DEADBAND || 
				OI.driverBackup.getRawButton(OI.XBOX_X_PORT) || newButtonHeld) && auto != null) {
			if (auto.isRunning()) {
				auto.cancel();
			}
		}
		

		Scheduler.getInstance().run();
		
		updateSmartDashboard();
		
		
//		if (shouldCrash) {
//			throw new RuntimeException("Current Sensor failed");
//		}
		
	}
	

	public void updateSmartDashboard() {
	
		if (debug) {
		SmartDashboard.putNumber("Yaw", RobotMap.pigeonDisplacementWrapper.pidGet());
		SmartDashboard.putNumber("Yaw Rate", RobotMap.pigeonRateWrapper.pidGet());
//		SmartDashboard.putNumber("velocity setpoint", RobotMap.driveTrain.velocitySetpointTN.getOutput());
//		SmartDashboard.putNumber("Yaw Rate Error", RobotMap.driveTrain.getAngleRateError());
		SmartDashboard.putNumber("Left Encoder Dist", RobotMap.driveLeftEncoderWrapperDistance.pidGet());
		SmartDashboard.putNumber("Right Encoder Dist", RobotMap.driveRightEncoderWrapperDistance.pidGet());
		SmartDashboard.putNumber("Left Encoder Rate", RobotMap.driveLeftEncoderWrapperRate.pidGet());
		SmartDashboard.putNumber("Right Encoder Rate", RobotMap.driveRightEncoderWrapperRate.pidGet());
//		SmartDashboard.putNumber("Velocity Error", RobotMap.driveTrain.getVelocityError());
//		SmartDashboard.putNumber("Distance Error", RobotMap.driveTrain.getDistError());
		SmartDashboard.putNumber("Angle Error", RobotMap.driveTrain.getAngleError());
		SmartDashboard.putNumber("Angle Setpoint", RobotMap.driveTrain.anglePID.getSetpoint());
		
		SmartDashboard.putNumber("Left Current", RobotMap.driveLeftTalon.getOutputCurrent());
		SmartDashboard.putNumber("Right Current", RobotMap.driveRightTalon.getOutputCurrent());

//	
//		SmartDashboard.putNumber("Distance PID Output", RobotMap.driveTrain.getDistancePIDOutput());
//		SmartDashboard.putNumber("Velocity PID Output", RobotMap.driveTrain.getVelocityPIDOutput());
//		SmartDashboard.putNumber("Left Current Output", RobotMap.driveLeftTalon.getMotorOutputPercent());
//		SmartDashboard.putNumber("Left Current", RobotMap.driveLeftTalon.getOutputCurrent());
//		SmartDashboard.putNumber("Right Current", RobotMap.driveRightTalon.getOutputCurrent());
//		SmartDashboard.putNumber("Relative Wrist", RobotMap.wristTalon.getSensorCollection().getQuadraturePosition());
//		SmartDashboard.putNumber("Relative Elbow", RobotMap.elbowTalon.getSensorCollection().getQuadraturePosition());
//		
//		SmartDashboard.putNumber("Left Current Error", RobotMap.driveLeftTalon.getClosedLoopError(0));
//		SmartDashboard.putNumber("Right Current Error", -RobotMap.driveRightTalon.getClosedLoopError(0));
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
		SmartDashboard.putNumber("X", RobotMap.deadReckoning.getX());
		SmartDashboard.putNumber("Y", RobotMap.deadReckoning.getY());
		SmartDashboard.putNumber("Elbow Current error", RobotMap.elbowTalon.getClosedLoopError(0));
		SmartDashboard.putNumber("Wrist Current Error", RobotMap.wristTalon.getClosedLoopError(0));
		SmartDashboard.putNumber("Wrist Current", RobotMap.wristTalon.getOutputCurrent());
		SmartDashboard.putNumber("Wrist Enc Rate", RobotMap.wristEncoderWrapperRate.pidGet());
		SmartDashboard.putNumber("Elbow Enc Rate", RobotMap.elbowEncoderWrapperRate.pidGet());
		SmartDashboard.putNumber("Wrist Ang Error", RobotMap.arm.getWristAngError());
		SmartDashboard.putNumber("Elbow Ang Vel Max Error", RobotMap.arm.getElbowAngVelMaxError());
		SmartDashboard.putNumber("Elbow Ang Vel Min Error", RobotMap.arm.getElbowAngVelMinError());
		SmartDashboard.putNumber("Elbow Ang Error", RobotMap.arm.getElbowAngError());
		SmartDashboard.putNumber("Elbow Enc Raw", RobotMap.elbowTalon.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Wrist Current", RobotMap.wristTalon.getOutputCurrent());
		SmartDashboard.putNumber("Elbow Current", RobotMap.elbowTalon.getOutputCurrent());
		SmartDashboard.putNumber("Intake Average Current", (RobotMap.intakeLeftTalon.getOutputCurrent() + RobotMap.intakeRightTalon.getOutputCurrent())/2);
//		SmartDashboard.putNumber("Angle TN", RobotMap.driveTrain.angleOutputTN.pidGet());
		//SmartDashboard.putNumber("Yaw Rate", RobotMap.pigeonRateWrapper.pidGet());
		SmartDashboard.putNumber("Drift", RobotMap.pathTracker.getDrift());
//		SmartDashboard.putNumber("X", RobotMap.deadReckoning.getX());
//		SmartDashboard.putNumber("Y", RobotMap.deadReckoning.getY());
		SmartDashboard.putNumber("Elbow Current Error", RobotMap.elbowTalon.getOutputCurrent());
//		SmartDashboard.putNumber("Drift", RobotMap.pathTracker.getDrift());
		
		
//		SmartDashboard.putNumber("Sonic Distance", RobotMap.sonic.getRangeInches());
//		SmartDashboard.putBoolean("Sonic", RobotMap.sonic.getRangeInches() < 5);
		
		
		

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
		SmartDashboard.putNumber("Wrist PercentOutput", RobotMap.wristTalon.getMotorOutputPercent());
		SmartDashboard.putNumber("Elbow PercentOutput", RobotMap.elbowTalon.getMotorOutputPercent());
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
		SmartDashboard.putNumber("Wrist Encoder Distance", RobotMap.wristEncoderWrapperDistance.pidGet());
		
		SmartDashboard.putNumber("Current Max Speed", RobotMap.driveTrain.distancePID.getMaxOutput());
		
//		
//		SmartDashboard.putNumber("X", RobotMap.deadReckoning.getX());
//		SmartDashboard.putNumber("Y", RobotMap.deadReckoning.getY());
		}
		
		
	    double avgEncoderDist = (RobotMap.driveLeftEncoderWrapperDistance.pidGet() + RobotMap.driveRightEncoderWrapperDistance.pidGet())/2;
		double avgEncoderRate = (RobotMap.driveLeftEncoderWrapperRate.pidGet() + RobotMap.driveRightEncoderWrapperRate.pidGet())/2;
		double elbowAngle = RobotMap.elbowEncoderWrapperDistance.pidGet();
		double elbowRate = RobotMap.elbowEncoderWrapperRate.pidGet();
		double wristAngle = RobotMap.wristEncoderWrapperDistance.pidGet();
		double wristRate = RobotMap.wristEncoderWrapperRate.pidGet();
		double gyroAngle = RobotMap.pigeonDisplacementWrapper.pidGet();
		double gyroRate = RobotMap.pigeonRateWrapper.pidGet();
		double xPos = RobotMap.deadReckoning.getX();
		double yPos = RobotMap.deadReckoning.getY();
		double joystickDriver = ThresholdHandler.deadbandAndScale(OI.driver.getRawAxis(OI.XBOX_RTRIGGER_PORT), OI.XBOX_TRIGGER_DEADBAND, 0, 1) 
    			- ThresholdHandler.deadbandAndScale(OI.driver.getRawAxis(OI.XBOX_LTRIGGER_PORT), OI.XBOX_TRIGGER_DEADBAND, 0, 1);
		double joystickOperator = OI.operator.getRawAxis(OI.XBOX_LYJOYSTICK_PORT);
		double elbowCurrent = RobotMap.elbowTalon.getOutputCurrent();
		double wristCurrent = RobotMap.wristTalon.getOutputCurrent();
		double avgDriveTrainCurrent = (RobotMap.driveLeftTalon.getOutputCurrent() + RobotMap.driveRightTalon.getOutputCurrent())/2;
		double elbowPwm = RobotMap.elbowTalon.getMotorOutputPercent();
		double wristPwm = RobotMap.wristTalon.getMotorOutputPercent();
		
		
		double[] theData = new double[] { xPos, yPos, avgEncoderDist, avgEncoderRate, elbowAngle, elbowRate, wristAngle, wristRate, gyroAngle, gyroRate, joystickDriver, joystickOperator, elbowCurrent, wristCurrent, elbowPwm, wristPwm, avgDriveTrainCurrent};
		
		
		AutoLogger.addDataPt(theData);
		
		
		
		
	}
	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		debug = true;
		RobotMap.arm.setIsClimb(false);
		if (OI.driver.getRawButton(OI.XBOX_A_PORT)) {
			RobotMap.arm.setElbowCurrent(-2);
			RobotMap.arm.setWristCurrent(0);
		} else if (OI.driver.getRawButton(OI.XBOX_B_PORT)) {
			RobotMap.arm.setWristCurrent(2);
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
		
		if (OI.driver.getRawButton(OI.XBOX_LBUMPER_PORT)) {
			RobotMap.climber.setManual(-.05);
		} else if (OI.driver.getRawButton(OI.XBOX_RBUMPER_PORT)) {
			RobotMap.climber.setManual(.05);
		} else {
			RobotMap.climber.setManual(0);
		}
		RobotMap.elbowEncoderWrapperDistance.setPosition(-.190);
		RobotMap.wristEncoderWrapperDistance.setPosition(0.416);
		isHomed = true;
	}
	

}
