package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.DriveWithControllers;
import org.usfirst.frc.team2485.robot.subsystems.Arm.ArmSetpoint;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.FastMath;
import org.usfirst.frc.team2485.util.LowPassFilter;
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
	private static final double CURRENT_LIMIT_ARM_UP = 30, CURRENT_LIMIT_ARM_DOWN = 80;

	public WarlordsPIDController distancePID = new WarlordsPIDController();
	public WarlordsPIDController anglePID = new WarlordsPIDController();
	private WarlordsPIDController velocityPID = new WarlordsPIDController();
	
	private RampRate velocityRampRate = new RampRate();
	public RampRate angRampRate = new RampRate();
	private RampRate throttleRampRate = new RampRate();

	public TransferNode distanceTN = new TransferNode(0);
	public TransferNode angleTN = new TransferNode(0);
	public TransferNode velocityTN = new TransferNode(0);
	public TransferNode velocitySetpointTN = new TransferNode(0);
	public TransferNode angleOutputTN = new TransferNode(0);
	public TransferNode angleOutputRampedTN = new TransferNode(0);
	public TransferNode curvatureSetpointTN = new TransferNode(0);
	public TransferNode lowPassFilterAngVelTN = new TransferNode(0);

	public PIDSourceWrapper kp_distancePIDSource = new PIDSourceWrapper();
	private PIDSourceWrapper encoderDistancePIDSource = new PIDSourceWrapper();
	private PIDSourceWrapper encoderAvgVelocityPIDSource = new PIDSourceWrapper();
	private PIDSourceWrapper leftCurrentPIDSource = new PIDSourceWrapper();
	private PIDSourceWrapper rightCurrentPIDSource = new PIDSourceWrapper();
	public PIDSourceWrapper minVelocityORSource = new PIDSourceWrapper();
	public PIDSourceWrapper maxVelocityORSource = new PIDSourceWrapper();
	private PIDSourceWrapper minAngleORSource = new PIDSourceWrapper();
	private PIDSourceWrapper maxAngleORSource = new PIDSourceWrapper();
		
	private LowPassFilter angVelFilter = new LowPassFilter();
	
	private PIDSourceWrapper angVelSource = new PIDSourceWrapper();

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
					FastMath.sqrt(2 * ConstantsIO.accelerationMax / Math.abs(distancePID.getError())));
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
			return -(getMaxCurrent() - Math.abs(angleOutputTN.pidGet()));
		});
		maxVelocityORSource.setPidSource(() -> {
			return getMaxCurrent() - Math.abs(angleOutputTN.pidGet());

		});

		velocityPID.setOutputs(velocityTN);
		velocityPID.setSources(encoderAvgVelocityPIDSource);
		velocityPID.setSetpointSource(velocitySetpointTN);
		velocityPID.setOutputSources(maxVelocityORSource, minVelocityORSource);

		velocityRampRate.setSetpointSource(distanceTN);
		velocityRampRate.setOutputs(velocitySetpointTN);
		
	
		angVelSource.setPidSource(() -> {
			return curvatureSetpointTN.pidGet() * encoderAvgVelocityPIDSource.pidGet();
		});
		
		minAngleORSource.setPidSource(() -> {
			return -10;
		});
		
		maxAngleORSource.setPidSource(() -> {
			return 10;
		});
		
		angVelFilter.setSetpointSource(RobotMap.pigeonRateWrapper);
		angVelFilter.setOutputs(lowPassFilterAngVelTN);
		
		anglePID.setSources(RobotMap.pigeonDisplacementWrapper);
		anglePID.setVelocitySetpointSources(angVelSource);
		anglePID.setOutputs(angleOutputTN);
		anglePID.setVelocitySource(lowPassFilterAngVelTN);
		anglePID.setOutputSources(maxAngleORSource, minAngleORSource);
		anglePID.setInputRange(0, 2 * Math.PI);
		anglePID.setContinuous(true);
		
		angRampRate.setSetpointSource(angleOutputTN);
		angRampRate.setOutputs(angleOutputRampedTN);
		
		leftCurrentPIDSource.setPidSource(() -> {
			return angleOutputRampedTN.pidGet() + velocityTN.pidGet();
		});
		
		rightCurrentPIDSource.setPidSource(() -> {
			return velocityTN.pidGet() - angleOutputRampedTN.pidGet();
		});
		
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
		
		double percentUp = (RobotMap.elbowEncoderWrapperDistance.pidGet() - ArmSetpoint.SWITCH.getElbowPos()) / 
				(ArmSetpoint.SCALE_HIGH_BACK.getElbowPos() - ArmSetpoint.SWITCH.getElbowPos());
		percentUp = Math.min(1, percentUp);
		
		double upRamp = percentUp * (ConstantsIO.kUpRamp_TeleopUp - ConstantsIO.kUpRamp_TeleopDown) + ConstantsIO.kUpRamp_TeleopDown;
		double downRamp = percentUp * (ConstantsIO.kDownRamp_TeleopUp - ConstantsIO.kDownRamp_TeleopDown) + ConstantsIO.kDownRamp_TeleopDown;
		RobotMap.driveRightPWM.setRampRate(upRamp, downRamp);
		RobotMap.driveLeftPWM.setRampRate(upRamp, downRamp);

		////	double I = percentUp * (CURRENT_LIMIT_ARM_UP - CURRENT_LIMIT_ARM_DOWN) + CURRENT_LIMIT_ARM_DOWN;
//		double speed = percentUp * (SPEED_LIMIT - 1) + 1;
//		boolean drivingForward = getAverageSpeed() > 0;
//		boolean throttleForward = throttle > 0;
//		double ramp;
//		if (drivingForward == throttleForward || getAverageSpeed() == 0) { // starting
//			if (drivingForward) {
//				ramp = ConstantsIO.kRamp_AcceleratingForward;				
//			} else {
//				ramp = ConstantsIO.kRamp_AcceleratingBackward;
//			}
//		} else { // stopping
//			if (drivingForward) {
//				ramp = ConstantsIO.kRamp_DeceleratingForward;				
//			} else {
//				ramp = ConstantsIO.kRamp_DeceleratingBackward;				
//			}		
//		}
//		throttleRampRate.setRampRates(ramp, 1000);
//		throttle = throttleRampRate.getNextValue(throttle); // don't enable just use here																		// (kind of sketchy but what here isn't)
		
		double leftPwm, rightPwm;

		double vmax = throttle;
		double angularPwm = 0;

		if (quickturn) {
			angularPwm = steering* Math.abs(steering);
		} else if (steering != 0){
			double tempThrottle = Math.abs(throttle);//, Math.abs(encoderAvgVelocityPIDSource.pidGet() / MAX_VELOCITY));
			//angularPwm = Math.abs(getAverageSpeed()) * steering;
			angularPwm = (tempThrottle * steering);
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
		RobotMap.driveLeftTalon.enableCurrentLimit(false);
		RobotMap.driveRightTalon.enableCurrentLimit(false);

		
		
		
		double speed = 1;

		if (quickturn) {
			speed = percentUp * (0.4 - 1) + 1;
			if (percentUp > .5) {
				speed *= Math.abs(steering);
			}

		}


		leftPwm *= speed;
		rightPwm *= speed;
		
//		RobotMap.driveLeftPWM.setRampRate(upRamp, downRamp);
//		RobotMap.driveRightPWM.setRampRate(upRamp, downRamp);
		

		RobotMap.driveLeftPWM.set(leftPwm);
		RobotMap.driveRightPWM.set(rightPwm);

	}

	public double getDistancePIDOutput() {
		return distanceTN.getOutput();
	}

	public double getVelocityPIDOutput() {
		return velocityTN.getOutput();
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
		RobotMap.driveTrain.angRampRate.setRampRates(100, 100);

		
		rightMotorSetter.disable();
		leftMotorSetter.disable();
		
		RobotMap.driveLeftPWM.emergencyStop();
		RobotMap.driveRightPWM.emergencyStop();

	}

	public void emergencyStop() {
		reset();
	}

	public double angVelOutput() {
		return angleOutputTN.getOutput();
	}

	public void setVelocities(double linearVel, double angVel) {

		anglePID.disable();
		angRampRate.disable();
		distancePID.disable();
		leftMotorSetter.enable();
		rightMotorSetter.enable();
		velocityPID.enable();
		velocityRampRate.enable();

		// allows us to set setpoints directly
		velocityRampRate.setSetpointSource(null);

		velocityRampRate.setSetpoint(linearVel);

	
	}


	public void setCurrents(double l, double r) {
		RobotMap.driveLeftCurrent.set(l);
		RobotMap.driveRightCurrent.set(r);
	}

	public boolean driveTo(double distance, double maxSpeed, double angle, double curvature, double toleranceDist, double toleranceAngle) {
		velocityPID.enable();
		anglePID.enable();
		angRampRate.enable();
		distancePID.enable();
		velocityRampRate.enable();
		leftMotorSetter.enable();
		rightMotorSetter.enable();
		angVelFilter.enable();
		anglePID.setSetpoint(angle);
		distancePID.setSetpoint(distance);
		curvatureSetpointTN.setOutput(curvature);
		distancePID.setAbsoluteTolerance(toleranceDist);
		anglePID.setAbsoluteTolerance(toleranceAngle);
		
		distancePID.setOutputRange(-maxSpeed, maxSpeed);

		return distancePID.isOnTarget() && anglePID.isOnTarget() && Math.abs(encoderAvgVelocityPIDSource.pidGet()) < LOW_ENC_RATE;
	}
	
	
	
	

	public void updateConstants() {
		for (TalonSRX t : RobotMap.driveTalons) {
			t.enableVoltageCompensation(false);
			t.enableCurrentLimit(false);
//			t.configContinuousCurrentLimit(ConstantsIO.IMax, 0);
//			t.configPeakCurrentLimit(ConstantsIO.IMax, 0);

		}
		anglePID.setPID(ConstantsIO.kP_DriveAngle, ConstantsIO.kI_DriveAngle, ConstantsIO.kD_DriveAngle, ConstantsIO.kF_DriveAngle);
		angVelFilter.setFilterCoefficient(ConstantsIO.filterCoefficient);
		velocityPID.setPID(ConstantsIO.kP_DriveVelocity, ConstantsIO.kI_DriveVelocity, ConstantsIO.kD_DriveVelocity,
				ConstantsIO.kF_DriveVelocity);
		angRampRate.setRampRates(100, 100);
		velocityRampRate.setRampRates(ConstantsIO.kUpRamp_Velocity, ConstantsIO.kDownRamp_Velocity);
		RobotMap.driveLeftPWM.setRampRate(ConstantsIO.kUpRamp_TeleopDown, ConstantsIO.kDownRamp_TeleopDown);
	}

	public void enablePID(boolean enable) {
		if (enable) {
			velocityPID.enable();
			anglePID.enable();
			distancePID.enable();
			velocityRampRate.enable();
			angRampRate.enable();
		} else {
			velocityPID.disable();
			anglePID.disable();
			angRampRate.disable();
			distancePID.disable();
			velocityRampRate.disable();
			distanceTN.setOutput(0);
			velocityTN.setOutput(0);
			angleOutputTN.setOutput(0);
			curvatureSetpointTN.setOutput(0);
			angleTN.setOutput(0);
			anglePID.setSetpoint(0);
			velocitySetpointTN.setOutput(0);
			angVelFilter.disable();
		}
	}

	public double getAverageEncoderDistance() {
		return encoderDistancePIDSource.pidGet();
	}

}
