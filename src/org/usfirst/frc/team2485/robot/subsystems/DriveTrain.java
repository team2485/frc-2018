package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.DriveWithControllers;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.MotorSetter;
import org.usfirst.frc.team2485.util.PIDSourceWrapper;
import org.usfirst.frc.team2485.util.PigeonWrapperRateAndAngle;
import org.usfirst.frc.team2485.util.RampRate;
import org.usfirst.frc.team2485.util.ScalingMax;
import org.usfirst.frc.team2485.util.PigeonWrapperRateAndAngle.Units;
import org.usfirst.frc.team2485.util.TransferNode;
import org.usfirst.frc.team2485.util.WarlordsPIDController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class DriveTrain extends Subsystem {

	public enum DriveSpeed {
		SLOW_SPEED_RATING, NORMAL_SPEED_RATING;

		public double getSpeedFactor() {

			switch (this) {
			case SLOW_SPEED_RATING:
				return 0.5;
			case NORMAL_SPEED_RATING:
				return 1.0;
			default:
				return 1.0;
			}
		}
	}

	private double driveSpeed = DriveSpeed.NORMAL_SPEED_RATING.getSpeedFactor();

	public static final double STEERING_DEADBAND = 0.25;
	public static final double THROTTLE_DEADBAND = 0.25;
	public static final double LOW_ENC_RATE = 2;

	private WarlordsPIDController distancePID = new WarlordsPIDController();
	private WarlordsPIDController anglePID = new WarlordsPIDController();
	private WarlordsPIDController velocityPID = new WarlordsPIDController();
	private WarlordsPIDController angVelocityPID = new WarlordsPIDController();

	private TransferNode distanceTN = new TransferNode(0);
	private TransferNode angleTN = new TransferNode(0);
	public TransferNode velocityTN = new TransferNode(0);
	public TransferNode angVelocityTN = new TransferNode(0);

	public TransferNode curvatureTN = new TransferNode(0);
	
	private ScalingMax velScalingMax = new ScalingMax();
	
	private RampRate velocityRamp = new RampRate();

	private PIDSourceWrapper kp_distancePIDSource = new PIDSourceWrapper();
	private PIDSourceWrapper kp_anglePIDSource = new PIDSourceWrapper();

	private PIDSourceWrapper encoderDistancePIDSource = new PIDSourceWrapper();
	private PIDSourceWrapper encoderAvgVelocityPIDSource = new PIDSourceWrapper();

	private PIDSourceWrapper leftCurrentPIDSource = new PIDSourceWrapper();
	private PIDSourceWrapper rightCurrentPIDSource = new PIDSourceWrapper();
	

	private PIDSourceWrapper angVelTargSource = new PIDSourceWrapper();

	private PIDSourceWrapper minVelocityORSource = new PIDSourceWrapper();
	private PIDSourceWrapper maxVelocityORSource = new PIDSourceWrapper();

	private PIDSourceWrapper minAngVelocityORSource = new PIDSourceWrapper();
	private PIDSourceWrapper maxAngVelocityORSource = new PIDSourceWrapper();

	private MotorSetter leftMotorSetter = new MotorSetter();
	private MotorSetter rightMotorSetter = new MotorSetter();

	public double getMaxVoltage() {
		double vBat = RobotMap.PDP.getVoltage();
		double Vl = RobotMap.driveLeftTalon.getMotorOutputPercent();
		double Vl2 = RobotMap.driveLeftVictor2.getMotorOutputPercent();
		double Vl3 = RobotMap.driveLeftVictor3.getMotorOutputPercent();
		double Vl4 = RobotMap.driveLeftVictor4.getMotorOutputPercent();
		double VlAvg = Math.abs((Vl + Vl2 + Vl3 + Vl4) / 4);

		double Vr = RobotMap.driveRightTalon.getMotorOutputPercent();
		double Vr2 = RobotMap.driveRightVictor2.getMotorOutputPercent();
		double Vr3 = RobotMap.driveRightVictor3.getMotorOutputPercent();
		double Vr4 = RobotMap.driveRightVictor4.getMotorOutputPercent();
		double VrAvg = Math.abs((Vr + Vr2 + Vr3 + Vr4) / 4);

		double iMax = ConstantsIO.IMax;

		double iL = RobotMap.driveRightTalon.getOutputCurrent();
		double iR = RobotMap.driveLeftTalon.getOutputCurrent();

		double v = vBat;

		if (iL > iMax / 2) {
			v = Math.min(v, VlAvg * iMax / iL);
		}
		if (iR > iMax / 2) {
			v = Math.min(v, VrAvg * iMax / iR);
		}
		return v;
	}

	public double getMaxCurrent() {
		double pwmL = RobotMap.driveLeftTalon.getMotorOutputPercent();
		;
		double pwmR = RobotMap.driveRightTalon.getMotorOutputPercent();

		double iL = RobotMap.driveLeftTalon.getOutputCurrent();
		double iR = RobotMap.driveRightTalon.getOutputCurrent();

		double i = ConstantsIO.IMax;

		if (pwmL > .5) {
			i = Math.min(i, iL / pwmL);
		}
		if (pwmR > .5) {
			i = Math.min(i, iR / pwmR);
		}

		return i;

	}

	public DriveTrain() {
		
		
		kp_distancePIDSource.setPidSource(() -> {
			return Math.min(ConstantsIO.kPMax_Distance,
					2 * ConstantsIO.accelerationMax / Math.abs(encoderAvgVelocityPIDSource.pidGet()));
		});

		kp_anglePIDSource.setPidSource(() -> {
			return Math.min(ConstantsIO.kPMax_Angle,
					ConstantsIO.kPMax_Angle / Math.abs(encoderAvgVelocityPIDSource.pidGet()));
		});

		// velocityPIDSource.setPidSource(() -> {
		// return ;
		// });
		//
		// deltaVelocityPIDSource.setPidSource(() -> {
		// if ()
		// });

		// distance
		// distancePIDSource.setPidSource(() -> {
		// return Math.min(ConstantsIO.kP_Distance,
		// (2*ConstantsIO.accelerationMax) /
		// ((RobotMap.driveLeftEncoderWrapperRate.pidGet() +
		// RobotMap.driveRightEncoderWrapperRate.pidGet()) / 2)) ;
		// });
		//

		encoderDistancePIDSource.setPidSource(() -> {
			return (RobotMap.driveLeftEncoderWrapperDistance.pidGet()
					+ RobotMap.driveRightEncoderWrapperDistance.pidGet()) / 2;
		});

		distancePID.setOutputs(distanceTN);
		distancePID.setSources(encoderDistancePIDSource);
		distancePID.setConstantsSources(kp_distancePIDSource, null, null, null);
		
		velScalingMax.setSources(distanceTN);
		velScalingMax.setOutputs(velocityRamp);
		
		
		velocityRamp.setOutputs(velocityPID);

		encoderAvgVelocityPIDSource.setPidSource(() -> {
			return (RobotMap.driveLeftEncoderWrapperRate.pidGet() + RobotMap.driveRightEncoderWrapperRate.pidGet()) / 2;
		});

		minVelocityORSource.setPidSource(() -> {
			return -(getMaxCurrent() - Math.abs(angVelocityTN.pidGet()));
		});
		maxVelocityORSource.setPidSource(() -> {
			return getMaxCurrent() - Math.abs(angVelocityTN.pidGet());

		});

		velocityPID.setOutputs(velocityTN);
		velocityPID.setSources(encoderAvgVelocityPIDSource);
		velocityPID.setSetpointSource(distanceTN);
		velocityPID.setOutputSources(maxVelocityORSource, minVelocityORSource);

		// angle

		anglePID.setOutputs(angleTN);
		anglePID.setSources(RobotMap.pigeonDisplacementWrapper);
		anglePID.setContinuous(true);
		anglePID.setInputRange(0, 2 * Math.PI);
		anglePID.setConstantsSources(kp_anglePIDSource, null, null, null);

		angVelTargSource.setPidSource(() -> {
			return angleTN.pidGet() + (curvatureTN.pidGet() * encoderAvgVelocityPIDSource.pidGet());
		});

		minAngVelocityORSource.setPidSource(() -> {
			return -getMaxCurrent();
		});
		maxAngVelocityORSource.setPidSource(() -> {
			return getMaxCurrent();
		});

		angVelocityPID.setSetpointSource(angVelTargSource);
		angVelocityPID.setOutputs(angVelocityTN);
		angVelocityPID.setSources(RobotMap.pigeonRateWrapper);
		angVelocityPID.setOutputSources(maxAngVelocityORSource, minAngVelocityORSource);

		leftCurrentPIDSource.setPidSource(() -> {
			return velocityTN.pidGet() + angVelocityTN.pidGet();
		});
		rightCurrentPIDSource.setPidSource(() -> {
			return velocityTN.pidGet() - angVelocityTN.pidGet();
		});

		leftMotorSetter.setSetpointSource(leftCurrentPIDSource);
		leftMotorSetter.setOutputs(RobotMap.driveLeftCurrent);
		rightMotorSetter.setSetpointSource(rightCurrentPIDSource);
		rightMotorSetter.setOutputs(RobotMap.driveRightCurrent);

		// RobotMap.driveLeftTalon1.config_kF(0, .01, 0);
		// RobotMap.driveRightTalon1.config_kF(0, .01, 0);

	}

	public void initDefaultCommand() {
		 setDefaultCommand(new DriveWithControllers());
	}

	public void simpleDrive(double throttle, double steering) {
		velocityPID.disable();
		angVelocityPID.disable();
		rightMotorSetter.disable();
		leftMotorSetter.disable();
	
		boolean quickTurn = OI.driver.getRawButton(OI.XBOX_X_PORT);
		
		if(!quickTurn) {
			steering *= Math.abs(throttle);
		}
		
		double left = throttle + steering;
		double right = throttle - steering;

		if (Math.abs(left) > 1) {
			right /= Math.abs(left);
			left /= Math.abs(left);
		}
		if (Math.abs(right) > 1) {
			left /= Math.abs(right);
			right /= Math.abs(right);
		}
		

		RobotMap.driveLeftPWM.set(left);
		RobotMap.driveRightPWM.set(right);
	}

	public double getDistancePIDOutput() {
		return distanceTN.getOutput();
	}

	public double getVelocityPIDOutput() {
		return velocityTN.getOutput();
	}

	public void WARLordsDrive(double throttle, double steering) {
		velocityPID.disable();
		angVelocityPID.disable();
		rightMotorSetter.disable();
		leftMotorSetter.disable();
	
		boolean quickTurn = OI.driver.getRawButton(OI.XBOX_X_PORT);
		
		if(!quickTurn) {
			steering *= Math.abs(throttle);
		}
		
		double left = throttle + steering;
		double right = throttle - steering;

		if (Math.abs(left) > 1) {
			right /= Math.abs(left);
			left /= Math.abs(left);
		}
		if (Math.abs(right) > 1) {
			left /= Math.abs(right);
			right /= Math.abs(right);
		}
		

		RobotMap.driveLeftPWM.set(left);
		RobotMap.driveRightPWM.set(right);
	}

	public double getAngleRateError() {
		return angVelocityPID.getAvgError();
	}

	public double getAngleError() {
		return anglePID.getAvgError();
	}

	public void setDriveSpeed(DriveSpeed speed) {
		driveSpeed = speed.getSpeedFactor();
	}

	public void zeroEncoders() {

		RobotMap.driveLeftEncoderWrapperDistance.reset();
		RobotMap.driveRightEncoderWrapperDistance.reset();
		System.out.println("Zero");

	}

	public double getVelocityError() {
		return velocityPID.getAvgError();
	}

	public double getDistError() {
		return distancePID.getAvgError();
	}

	public void reset() {

		System.out.println("reset");
		velocityPID.disable();
		angVelocityPID.disable();
		rightMotorSetter.disable();
		leftMotorSetter.disable();
		distancePID.disable();
		anglePID.disable();
		velScalingMax.disable();
		velocityRamp.disable();
		velocityTN.setOutput(0);
		curvatureTN.setOutput(0);
		RobotMap.driveLeftPWM.set(0);
		RobotMap.driveRightPWM.set(0);
		RobotMap.driveLeftCurrent.set(0);
		RobotMap.driveLeftCurrent.set(0);

	}

	public void emergencyStop() {
		reset();
	}

	public void setVelocities(double linearVel, double angVel) {
		velScalingMax.enable();
		velocityRamp.enable();
		velocityPID.enable();
		angVelocityPID.enable();
		anglePID.disable();
		distancePID.disable();
		leftMotorSetter.enable();
		rightMotorSetter.enable();

		// allows us to set setpoints directly
		velocityPID.setSetpointSource(null);
		angVelocityPID.setSetpointSource(null);

		velocityPID.setSetpoint(linearVel);
		angVelocityPID.setSetpoint(angVel);
	}

	public void setCurrents(double l, double r) {
		RobotMap.driveLeftCurrent.set(l);
		RobotMap.driveRightCurrent.set(r);
	}

	public boolean driveTo(double distance, double maxSpeed, double angle, double curvature, double tolerance) {
		velocityRamp.enable();
		velScalingMax.enable();
		velocityPID.enable();
		angVelocityPID.enable();
		anglePID.enable();
		distancePID.enable();
		leftMotorSetter.enable();
		rightMotorSetter.enable();
		anglePID.setSetpoint(angle);
		distancePID.setSetpoint(distance);
		curvatureTN.setOutput(curvature);
		distancePID.setAbsoluteTolerance(tolerance);

		System.out.println("Driving");

		distancePID.setOutputRange(-maxSpeed, maxSpeed);
		anglePID.setOutputRange(-maxSpeed / RobotMap.ROBOT_WIDTH, maxSpeed / RobotMap.ROBOT_WIDTH);

		return distancePID.isOnTarget() && Math.abs(encoderAvgVelocityPIDSource.pidGet()) < LOW_ENC_RATE;
	}

	public void updateConstants() {
		for (TalonSRX driveTalon : RobotMap.driveTalons) {
			driveTalon.enableVoltageCompensation(false);
			// driveTalon.configVoltageCompSaturation(ConstantsIO.voltageMax, 0);
//			driveTalon.enableCurrentLimit(false);
			// driveTalon.configContinuousCurrentLimit(ConstantsIO.IMax, 0);
			// driveTalon.configOpenloopRamp(.2, 0);
		}
		
	
		RobotMap.driveLeftTalon.configContinuousCurrentLimit(10, 0);
		RobotMap.driveLeftTalon.configPeakCurrentLimit(0, 0);
		RobotMap.driveRightTalon.configContinuousCurrentLimit(5, 0);
		RobotMap.driveRightTalon.configPeakCurrentLimit(0, 0);
		RobotMap.driveLeftTalon.enableCurrentLimit(false);
		RobotMap.driveRightTalon.enableCurrentLimit(false);
		
		
		System.out.println("update constants");

		
		velocityPID.setPID(ConstantsIO.kP_DriveVelocity, ConstantsIO.kI_DriveVelocity, ConstantsIO.kD_DriveVelocity,
				ConstantsIO.kF_DriveVelocity);
		angVelocityPID.setPID(ConstantsIO.kP_DriveAngVel, ConstantsIO.kI_DriveAngVel, ConstantsIO.kD_DriveAngVel,
				ConstantsIO.kF_DriveAngVel);
		velocityRamp.setRampRates(ConstantsIO.kUpRamp_DriveVelocity, ConstantsIO.kDownRamp_DriveVelocity);

	}

	public double getAverageEncoderDistance() {
		// TODO Auto-generated method stub
		return encoderDistancePIDSource.pidGet();
	}
}