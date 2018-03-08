package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.DriveWithControllers;
import org.usfirst.frc.team2485.robot.subsystems.Arm.ArmSetpoint;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.MotorSetter;
import org.usfirst.frc.team2485.util.PIDSourceWrapper;
import org.usfirst.frc.team2485.util.RampRate;
import org.usfirst.frc.team2485.util.TransferNode;
import org.usfirst.frc.team2485.util.WarlordsPIDController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class DriveTrain extends Subsystem {

	// constants
	public static final double LOW_ENC_RATE = 2;
	private static final double SPEED_LIMIT = .75;
	private static final double CURRENT_LIMIT_ARM_UP = 20, CURRENT_LIMIT_ARM_DOWN = 80;
	private static final double MAX_VELOCITY = 180;

	public WarlordsPIDController distancePID = new WarlordsPIDController();
	private WarlordsPIDController anglePID = new WarlordsPIDController();
	private WarlordsPIDController velocityPID = new WarlordsPIDController();
	private WarlordsPIDController angularVelocityPID = new WarlordsPIDController();
	public WarlordsPIDController driveStraightPID = new WarlordsPIDController();
	
	
	private RampRate velocityRampRate = new RampRate();
	public RampRate angularVelocityRampRate = new RampRate();

	public TransferNode driveStraightTN = new TransferNode(0);
	public TransferNode angleSetpointTN = new TransferNode(0);
	private TransferNode distanceTN = new TransferNode(0);
	private TransferNode angleTN = new TransferNode(0);
	public TransferNode velocityTN = new TransferNode(0);
	public TransferNode velocitySetpointTN = new TransferNode(0);
	public TransferNode angularVelocityTN = new TransferNode(0);
	public TransferNode angularVelocitySetpointTN = new TransferNode(0);
	public TransferNode curvatureSetpointTN = new TransferNode(0);

	private PIDSourceWrapper kp_distancePIDSource = new PIDSourceWrapper();
	private PIDSourceWrapper kp_anglePIDSource = new PIDSourceWrapper();
	private PIDSourceWrapper anglePIDSetpointSource = new PIDSourceWrapper();
	private PIDSourceWrapper encoderDistancePIDSource = new PIDSourceWrapper();
	private PIDSourceWrapper encoderAvgVelocityPIDSource = new PIDSourceWrapper();
	private PIDSourceWrapper curvaturePIDSource = new PIDSourceWrapper();
	private PIDSourceWrapper leftCurrentPIDSource = new PIDSourceWrapper();
	private PIDSourceWrapper rightCurrentPIDSource = new PIDSourceWrapper();
	private PIDSourceWrapper curvatureSetpointSource = new PIDSourceWrapper();
	public PIDSourceWrapper minVelocityORSource = new PIDSourceWrapper();
	public PIDSourceWrapper maxVelocityORSource = new PIDSourceWrapper();
	private PIDSourceWrapper minAngVelocityORSource = new PIDSourceWrapper();
	private PIDSourceWrapper maxAngVelocityORSource = new PIDSourceWrapper();

	private MotorSetter leftMotorSetter = new MotorSetter();
	private MotorSetter rightMotorSetter = new MotorSetter();

	public double getMaxCurrent() {
		double pwmL = RobotMap.driveLeftTalon.getMotorOutputPercent();
		;
		double pwmR = RobotMap.driveRightTalon.getMotorOutputPercent();

		double iL = RobotMap.driveLeftTalon.getOutputCurrent();
		double iR = RobotMap.driveRightTalon.getOutputCurrent();

		double i = ConstantsIO.IMax;

		if (Math.abs(pwmL) > .5) {
			i = Math.min(i, Math.abs(iL / pwmL));
		}
		if (Math.abs(pwmR) > .5) {
			i = Math.min(i, Math.abs(iR / pwmR));
		}

		return i;
	}

	public DriveTrain() {

		kp_distancePIDSource.setPidSource(() -> {
			return Math.min(ConstantsIO.kPMax_Distance,
					Math.sqrt(2 * ConstantsIO.accelerationMax / Math.abs(distancePID.getAvgError())));
		});
		kp_anglePIDSource.setPidSource(() -> {
			return Math.min(ConstantsIO.kP_DriveAngleMax,
					ConstantsIO.kP_DriveAngle / Math.abs(encoderAvgVelocityPIDSource.pidGet()));
		});

		anglePIDSetpointSource.setPidSource(() -> {
			return angleSetpointTN.pidGet() /*+ RobotMap.pathTracker.getDrift() * ConstantsIO.kP_Drift**/;
		});

		encoderDistancePIDSource.setPidSource(() -> {
			return (RobotMap.driveLeftEncoderWrapperDistance.pidGet()
					+ RobotMap.driveRightEncoderWrapperDistance.pidGet()) / 2;
		});

		distancePID.setOutputs(distanceTN);
		distancePID.setSources(encoderDistancePIDSource);
		distancePID.setConstantsSources(kp_distancePIDSource, null, null, null);
		
		

		encoderAvgVelocityPIDSource.setPidSource(() -> {
			return (RobotMap.driveLeftEncoderWrapperRate.pidGet() + RobotMap.driveRightEncoderWrapperRate.pidGet()) / 2;
		});

		minVelocityORSource.setPidSource(() -> {
			return -(getMaxCurrent() - Math.abs(angularVelocityTN.pidGet()));
		});
		maxVelocityORSource.setPidSource(() -> {
			return getMaxCurrent() - Math.abs(angularVelocityTN.pidGet());

		});

		velocityPID.setOutputs(velocityTN);
		velocityPID.setSources(encoderAvgVelocityPIDSource);
		velocityPID.setSetpointSource(velocitySetpointTN);
		velocityPID.setOutputSources(maxVelocityORSource, minVelocityORSource);

		velocityRampRate.setSetpointSource(distanceTN);
		velocityRampRate.setOutputs(velocitySetpointTN);
	

		anglePID.setOutputs(angleTN);
		anglePID.setSources(RobotMap.pigeonDisplacementWrapper);
		anglePID.setSetpointSource(anglePIDSetpointSource);
		anglePID.setContinuous(true);
		anglePID.setInputRange(0, 2 * Math.PI);

		minAngVelocityORSource.setPidSource(() -> {
			return  -getMaxCurrent();
		});
		maxAngVelocityORSource.setPidSource(() -> {
			return getMaxCurrent();
		});
		
		angularVelocityRampRate.setSetpointSource(angleTN);
		angularVelocityRampRate.setOutputs(angularVelocitySetpointTN);
		

		curvaturePIDSource.setPidSource(() -> {
			if (Math.abs(encoderAvgVelocityPIDSource.pidGet()) > LOW_ENC_RATE) {
				return RobotMap.pigeonRateWrapper.pidGet() / Math.abs(encoderAvgVelocityPIDSource.pidGet());
			} else {
				return 0;
			}
		});

		angularVelocityPID.setSetpointSource(curvatureSetpointSource);
		angularVelocityPID.setOutputs(angularVelocityTN);
		angularVelocityPID.setSources(RobotMap.pigeonRateWrapper);
		angularVelocityPID.setOutputSources(maxAngVelocityORSource, minAngVelocityORSource);

		curvatureSetpointSource.setPidSource(() -> {
			return angularVelocitySetpointTN.getOutput() + curvatureSetpointTN.getOutput() * encoderAvgVelocityPIDSource.pidGet();
		});

		leftCurrentPIDSource.setPidSource(() -> {
			return velocityTN.pidGet() + angularVelocityTN.pidGet();
		});
		rightCurrentPIDSource.setPidSource(() -> {
			return velocityTN.pidGet() - angularVelocityTN.pidGet();
		});

		driveStraightPID.setOutputs(driveStraightTN);
		driveStraightPID.setSources(RobotMap.pigeonRateWrapper);
		driveStraightPID.setOutputRange(-.1, .1);
		
		leftMotorSetter.setSetpointSource(leftCurrentPIDSource);
		leftMotorSetter.setOutputs(RobotMap.driveLeftCurrent);
		rightMotorSetter.setSetpointSource(rightCurrentPIDSource);
		rightMotorSetter.setOutputs(RobotMap.driveRightCurrent);

	}

	public void initDefaultCommand() {
		setDefaultCommand(new DriveWithControllers());
	}
	
	public double getAverageSpeed() {
		return (RobotMap.driveLeftEncoderWrapperRate.pidGet() + RobotMap.driveRightEncoderWrapperRate.pidGet()) / 2;
	}

//	public void simpleDrive(double throttle, double steering, boolean quickturn) {
//		enablePID(false);
//
//		double leftPwm, rightPwm;
//
//		double vmax = throttle;
//
//		double angularPwm = 0;
//
//		if (quickturn) {
//			angularPwm = steering * Math.abs(steering);
//		} else {
//			double tempThrottle = Math.max(Math.abs(throttle), Math.abs(encoderAvgVelocityPIDSource.pidGet() / MAX_VELOCITY));
//			//angularPwm = Math.abs(getAverageSpeed()) * steering;
//			angularPwm = (tempThrottle * steering);
//		} 
//		//
//		//		if (vmax > 0) {
//		//			vmax -= Math.abs(angularPwm);
//		//		} else if (vmax < 0) {
//		//			vmax += Math.abs(angularPwm);
//		//		}
//
//		leftPwm = vmax + angularPwm;
//		rightPwm = vmax - angularPwm;
//
//		if(Math.abs(leftPwm) > 1) {
//			//squared to the fifth exponent
//			rightPwm /= Math.abs(leftPwm);
//			leftPwm /= Math.abs(leftPwm);
//		} else if (Math.abs(rightPwm) > 1) {
//			leftPwm /= Math.abs(rightPwm);
//			rightPwm /= Math.abs(rightPwm);
//		}
//
//		RobotMap.driveLeftTalon.enableVoltageCompensation(false);
//		RobotMap.driveRightTalon.enableVoltageCompensation(false);
//		RobotMap.driveLeftTalon.enableCurrentLimit(true);
//		RobotMap.driveRightTalon.enableCurrentLimit(true);
//
//		double percentUp = (RobotMap.elbowEncoderWrapperDistance.pidGet() - ArmSetpoint.SWITCH.getElbowPos()) / 
//				(0.25 - ArmSetpoint.SWITCH.getElbowPos());
//		double I = percentUp * (CURRENT_LIMIT_ARM_UP - CURRENT_LIMIT_ARM_DOWN) + CURRENT_LIMIT_ARM_DOWN;
//		double speed = percentUp * (SPEED_LIMIT - 1) + 1;
//
//		if(I < 24) {
//			RobotMap.driveLeftTalon.configContinuousCurrentLimit((int) (I/2), 0);
//			RobotMap.driveRightTalon.configContinuousCurrentLimit((int) (I/2), 0);
//			RobotMap.driveLeftVictor3.set(ControlMode.PercentOutput, 0);
//			RobotMap.driveLeftVictor4.set(ControlMode.PercentOutput, 0);
//			RobotMap.driveRightVictor3.set(ControlMode.PercentOutput, 0);
//			RobotMap.driveRightVictor4.set(ControlMode.PercentOutput, 0);
//		} else {
//			RobotMap.driveLeftTalon.configContinuousCurrentLimit((int) (I/4), 0);
//			RobotMap.driveRightTalon.configContinuousCurrentLimit((int) (I/4), 0);
//			RobotMap.driveLeftVictor3.follow(RobotMap.driveLeftTalon);
//			RobotMap.driveLeftVictor4.follow(RobotMap.driveLeftTalon);
//			RobotMap.driveRightVictor3.follow(RobotMap.driveRightTalon);
//			RobotMap.driveRightVictor4.follow(RobotMap.driveRightTalon);
//		}
//
//
//		leftPwm *= speed;
//		rightPwm *= speed;
//
//		RobotMap.driveLeftPWM.set(leftPwm);
//		RobotMap.driveRightPWM.set(rightPwm);
//	}

	public void WARlordsDrive(double throttle, double steering, boolean quickturn) {
		enablePID(false);

		
		double leftPwm, rightPwm;

		double vmax = throttle;

		double angularPwm = 0;

		if (quickturn) {
			angularPwm = steering* Math.abs(steering);
			driveStraightPID.disable();
		} else if (steering != 0){
			double tempThrottle = Math.max(Math.abs(throttle), Math.abs(encoderAvgVelocityPIDSource.pidGet() / MAX_VELOCITY));
			//angularPwm = Math.abs(getAverageSpeed()) * steering;
			angularPwm = (tempThrottle * steering);
			driveStraightPID.disable();
		} else if (Math.abs(getAverageSpeed()) > LOW_ENC_RATE){
			driveStraightPID.enable();
			driveStraightPID.setSetpoint(0);
			angularPwm = driveStraightTN.pidGet();
			
		} else {
			driveStraightPID.disable();
		}

		leftPwm = vmax + angularPwm;
		rightPwm = vmax - angularPwm;

		if(Math.abs(leftPwm) > 1) {
			//squared to the fifth exponent
			rightPwm /= Math.abs(leftPwm);
			leftPwm /= Math.abs(leftPwm);
		} else if (Math.abs(rightPwm) > 1) {
			leftPwm /= Math.abs(rightPwm);
			rightPwm /= Math.abs(rightPwm);
		}

		RobotMap.driveLeftTalon.enableVoltageCompensation(false);
		RobotMap.driveRightTalon.enableVoltageCompensation(false);
		RobotMap.driveLeftTalon.enableCurrentLimit(true);
		RobotMap.driveRightTalon.enableCurrentLimit(true);

		double percentUp = (RobotMap.elbowEncoderWrapperDistance.pidGet() - ArmSetpoint.SWITCH.getElbowPos()) / 
				(0.25 - ArmSetpoint.SWITCH.getElbowPos());
		double I = percentUp * (CURRENT_LIMIT_ARM_UP - CURRENT_LIMIT_ARM_DOWN) + CURRENT_LIMIT_ARM_DOWN;
		double speed = percentUp * (SPEED_LIMIT - 1) + 1;

		if(I < 24) {
			RobotMap.driveLeftTalon.configContinuousCurrentLimit((int) (I/2), 0);
			RobotMap.driveRightTalon.configContinuousCurrentLimit((int) (I/2), 0);
			RobotMap.driveLeftVictor3.set(ControlMode.PercentOutput, 0);
			RobotMap.driveLeftVictor4.set(ControlMode.PercentOutput, 0);
			RobotMap.driveRightVictor3.set(ControlMode.PercentOutput, 0);
			RobotMap.driveRightVictor4.set(ControlMode.PercentOutput, 0);
		} else {
			RobotMap.driveLeftTalon.configContinuousCurrentLimit((int) (I/4), 0);
			RobotMap.driveRightTalon.configContinuousCurrentLimit((int) (I/4), 0);
			RobotMap.driveLeftVictor3.follow(RobotMap.driveLeftTalon);
			RobotMap.driveLeftVictor4.follow(RobotMap.driveLeftTalon);
			RobotMap.driveRightVictor3.follow(RobotMap.driveRightTalon);
			RobotMap.driveRightVictor4.follow(RobotMap.driveRightTalon);
		}


		leftPwm *= speed;
		rightPwm *= speed;

		RobotMap.driveLeftPWM.set(leftPwm);
		RobotMap.driveRightPWM.set(rightPwm);

	}

	public double getDistancePIDOutput() {
		return distanceTN.getOutput();
	}

	public double getVelocityPIDOutput() {
		return velocityTN.getOutput();
	}

	public double getAngleRateError() {
		return angularVelocityPID.getAvgError();
	}

	public double getAngleError() {
		return anglePID.getAvgError();
	}

	public void zeroEncoders() {

		RobotMap.driveLeftEncoderWrapperDistance.reset();
		RobotMap.driveRightEncoderWrapperDistance.reset();

	}

	public double getVelocityError() {
		return velocityPID.getAvgError();
	}

	public double getDistError() {
		return distancePID.getAvgError();
	}

	public void reset() {

		enablePID(false);
		RobotMap.driveLeftPWM.set(0);
		RobotMap.driveRightPWM.set(0);

		rightMotorSetter.disable();
		leftMotorSetter.disable();

	}

	public void emergencyStop() {
		reset();
	}

	public double angVelOutput() {
		return angularVelocityTN.getOutput();
	}

	public void setVelocities(double linearVel, double angVel) {

		anglePID.disable();
		distancePID.disable();
		leftMotorSetter.enable();
		rightMotorSetter.enable();
		velocityPID.enable();
		velocityRampRate.enable();
		angularVelocityRampRate.enable();

		// allows us to set setpoints directly
		velocityRampRate.setSetpointSource(null);
		angularVelocityRampRate.setSetpointSource(null);
		angularVelocityPID.setSetpointSource(null);

		velocityRampRate.setSetpoint(linearVel);

		angularVelocityPID.enable();
		angularVelocityPID.setSetpoint(angVel);

	}


	public void setCurrents(double l, double r) {
		RobotMap.driveLeftCurrent.set(l);
		RobotMap.driveRightCurrent.set(r);
	}

	public boolean driveTo(double distance, double maxSpeed, double angle, double curvature, double toleranceDist, double toleranceAngle) {
		velocityPID.enable();
		anglePID.enable();
		distancePID.enable();
		velocityRampRate.enable();
		angularVelocityRampRate.enable();
		leftMotorSetter.enable();
		rightMotorSetter.enable();
		angleSetpointTN.setOutput(angle);
		distancePID.setSetpoint(distance);
		curvatureSetpointTN.setOutput(curvature);
		distancePID.setAbsoluteTolerance(toleranceDist);
		anglePID.setAbsoluteTolerance(toleranceAngle);
		angularVelocityPID.enable();

		distancePID.setOutputRange(-maxSpeed, maxSpeed);
		anglePID.setOutputRange(-3, 3);

		return distancePID.isOnTarget() && anglePID.isOnTarget() && Math.abs(encoderAvgVelocityPIDSource.pidGet()) < LOW_ENC_RATE;
	}

	public void updateConstants() {
		for (TalonSRX t : RobotMap.driveTalons) {
			t.enableVoltageCompensation(false);
			t.enableCurrentLimit(true);
			t.configContinuousCurrentLimit(ConstantsIO.IMax, 0);
			t.configPeakCurrentLimit(ConstantsIO.IMax, 0);

		}
		anglePID.setPID(ConstantsIO.kP_DriveAngle, 0, 0);
		velocityPID.setPID(ConstantsIO.kP_DriveVelocity, ConstantsIO.kI_DriveVelocity, ConstantsIO.kD_DriveVelocity,
				ConstantsIO.kF_DriveVelocity);
		angularVelocityPID.setPID(ConstantsIO.kP_DriveAngVel, ConstantsIO.kI_DriveAngVel, ConstantsIO.kD_DriveAngVel,
				ConstantsIO.kF_DriveAngVel);
		velocityRampRate.setRampRates(ConstantsIO.kUpRamp_Velocity, ConstantsIO.kDownRamp_Velocity);
		angularVelocityRampRate.setRampRates(100, 100);
		driveStraightPID.setPID(ConstantsIO.kP_DriveStraight, ConstantsIO.kI_DriveStraight, 0);
	}

	public void enablePID(boolean enable) {
		if (enable) {
			velocityPID.enable();
			angularVelocityPID.enable();
			anglePID.enable();
			distancePID.enable();
			velocityRampRate.enable();
			angularVelocityRampRate.enable();
		} else {
			velocityPID.disable();
			angularVelocityPID.disable();
			anglePID.disable();
			distancePID.disable();
			velocityRampRate.disable();
			angularVelocityRampRate.disable();
			distanceTN.setOutput(0);
			velocityTN.setOutput(0);
			angularVelocityTN.setOutput(0);
			curvatureSetpointTN.setOutput(0);
			angleTN.setOutput(0);
			angleSetpointTN.setOutput(0);
			velocitySetpointTN.setOutput(0);
		}
	}

	public double getAverageEncoderDistance() {
		return encoderDistancePIDSource.pidGet();
	}

}
