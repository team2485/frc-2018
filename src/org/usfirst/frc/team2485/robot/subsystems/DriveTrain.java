package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.DriveWithControllers;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.MotorSetter;
import org.usfirst.frc.team2485.util.PIDSourceWrapper;
import org.usfirst.frc.team2485.util.PigeonWrapperRateAndAngle;
import org.usfirst.frc.team2485.util.PigeonWrapperRateAndAngle.Units;
import org.usfirst.frc.team2485.util.ThresholdHandler;
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
	private WarlordsPIDController curvaturePID = new WarlordsPIDController();
	private WarlordsPIDController angVelTeleopPID = new WarlordsPIDController();

	
	private TransferNode distanceTN = new TransferNode(0);
	private TransferNode angleTN = new TransferNode(0);
	public TransferNode velocityTN = new TransferNode(0);
	public TransferNode curvatureTN = new TransferNode(0);

	private TransferNode curvatureSetpointTN = new TransferNode(0);
	private TransferNode angVelTeleopTN = new TransferNode(0);

	private PIDSourceWrapper kp_distancePIDSource = new PIDSourceWrapper();

	private PIDSourceWrapper encoderDistancePIDSource = new PIDSourceWrapper();
	private PIDSourceWrapper encoderAvgVelocityPIDSource = new PIDSourceWrapper();
	private PIDSourceWrapper curvaturePIDSource = new PIDSourceWrapper();

	private PIDSourceWrapper leftCurrentPIDSource = new PIDSourceWrapper();
	private PIDSourceWrapper rightCurrentPIDSource = new PIDSourceWrapper();

	private PIDSourceWrapper curvatureSetpointSource = new PIDSourceWrapper();
	
	private PIDSourceWrapper angVelOutputSource = new PIDSourceWrapper();
	
	private PIDSourceWrapper minVelocityORSource = new PIDSourceWrapper();
	private PIDSourceWrapper maxVelocityORSource = new PIDSourceWrapper();
	
	private PIDSourceWrapper minAngVelocityORSource = new PIDSourceWrapper();
	private PIDSourceWrapper maxAngVelocityORSource = new PIDSourceWrapper();


	private MotorSetter leftMotorSetter = new MotorSetter();
	private MotorSetter rightMotorSetter = new MotorSetter();
	

	
	public double getMaxCurrent() {
		double pwmL = RobotMap.driveLeftTalon.getMotorOutputPercent();;
		double pwmR = RobotMap.driveRightTalon.getMotorOutputPercent();

		double iL = RobotMap.driveRightTalon.getOutputCurrent();
		double iR = RobotMap.driveLeftTalon.getOutputCurrent();
		
		double i = ConstantsIO.IMax;
		
		if (pwmL > .5) {
			i = Math.min(i, iL/pwmL);
		}
		if (pwmR > .5) {
			i = Math.min(i, iR/pwmR);
		}
		
		return i;
	}

	public DriveTrain() {
		kp_distancePIDSource.setPidSource(() -> {
			return Math.min(ConstantsIO.kPMax_Distance, 2*ConstantsIO.accelerationMax/Math.abs(encoderAvgVelocityPIDSource.pidGet()));
		});
		//    	velocityPIDSource.setPidSource(() -> {
		//    		return ;
		//    	});
		//
		//    	deltaVelocityPIDSource.setPidSource(() -> {
		//    		if ()
		//    	});


		//distance
		//    	distancePIDSource.setPidSource(() -> {
		//    		return Math.min(ConstantsIO.kP_Distance, 
		//    				(2*ConstantsIO.accelerationMax) / 
		//    				((RobotMap.driveLeftEncoderWrapperRate.pidGet() + RobotMap.driveRightEncoderWrapperRate.pidGet()) / 2)) ;
		//    	});
		//    	
		
//		curvaturePIDSource.setPidSource(() -> {
//			if (Math.abs(encoderAvgVelocityPIDSource.pidGet()) > LOW_ENC_RATE) {
//				return RobotMap.pigeonRateWrapper.pidGet() / encoderAvgVelocityPIDSource.pidGet();
//			} else {
//				return 0;
//			}
//		});

		encoderDistancePIDSource.setPidSource(() -> {
			return (RobotMap.driveLeftEncoderWrapperDistance.pidGet() + RobotMap.driveRightEncoderWrapperDistance.pidGet()) / 2;
		});

		distancePID.setOutputs(distanceTN);
		distancePID.setSources(encoderDistancePIDSource);
		distancePID.setConstantsSources(kp_distancePIDSource, null, null, null);

		encoderAvgVelocityPIDSource.setPidSource(() -> {
			return (RobotMap.driveLeftEncoderWrapperRate.pidGet() + RobotMap.driveRightEncoderWrapperRate.pidGet()) / 2;
		});
		
		minVelocityORSource.setPidSource(() -> {
			return -(getMaxCurrent() - Math.abs(curvatureTN.pidGet() * encoderAvgVelocityPIDSource.pidGet()));
		});
		maxVelocityORSource.setPidSource(() -> {
			return getMaxCurrent() - Math.abs(curvatureTN.pidGet() * encoderAvgVelocityPIDSource.pidGet());

		});
		
		velocityPID.setOutputs(velocityTN);
		velocityPID.setSources(encoderAvgVelocityPIDSource);
		velocityPID.setSetpointSource(distanceTN);
		velocityPID.setOutputSources(maxVelocityORSource, minVelocityORSource);
		

		//angle

		anglePID.setOutputs(angleTN);
//		anglePID.setSources(RobotMap.pigeonDisplacementWrapper);
		anglePID.setContinuous(true);
		anglePID.setInputRange(0, 2 * Math.PI);
  
		
		
		
		
		minAngVelocityORSource.setPidSource(() -> {
			return -getMaxCurrent();
		});
		maxAngVelocityORSource.setPidSource(() -> {
			return getMaxCurrent();
		});
	
		
		

		curvaturePID.setSetpointSource(curvatureSetpointSource);
		curvaturePID.setOutputs(curvatureTN);
		curvaturePID.setSources(curvaturePIDSource);
//		curvaturePID.setOutputSources(maxAngVelocityORSource, minAngVelocityORSource);
		
		
		angVelTeleopPID.setOutputs(angVelTeleopTN);
//		angVelTeleopPID.setSources(RobotMap.pigeonRateWrapper);
		
		
		curvatureSetpointSource.setPidSource(() -> {
			return angleTN.getOutput() + curvatureSetpointTN.getOutput();
		});
		
		leftCurrentPIDSource.setPidSource(() -> {
			return velocityTN.pidGet() + curvatureTN.pidGet() * encoderAvgVelocityPIDSource.pidGet();
		});
		rightCurrentPIDSource.setPidSource(() -> {
			return velocityTN.pidGet() - curvatureTN.pidGet() * encoderAvgVelocityPIDSource.pidGet();
		});

		leftMotorSetter.setSetpointSource(leftCurrentPIDSource);
		leftMotorSetter.setOutputs(RobotMap.driveLeftCurrent);
		rightMotorSetter.setSetpointSource(rightCurrentPIDSource);
		rightMotorSetter.setOutputs(RobotMap.driveRightCurrent);
		
//		RobotMap.driveLeftTalon1.config_kF(0, .01, 0);
//		RobotMap.driveRightTalon1.config_kF(0, .01, 0);


	}

	public void initDefaultCommand() {
		setDefaultCommand(new DriveWithControllers());
	}

	public void simpleDrive(double throttle, double steering) {
		
		throttle = mapPWM(throttle, THROTTLE_DEADBAND);
		steering = mapPWM(steering, STEERING_DEADBAND);
		
		enablePID(false);
		double left = throttle + steering;
		double right = throttle - steering;

		System.out.println(throttle);
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
	
	public double mapPWM(double pwm, double deadband) {	
		return ThresholdHandler.deadbandAndScale(pwm, deadband, 0, 1);
	}

	/* public void WARLordsDrive(double throttle, double steering , boolean quickturn) {
		velocityPID.disable();
		distancePID.disable();
		anglePID.disable();
		curvaturePID.disable();
		
		throttle = mapPWM(throttle, THROTTLE_DEADBAND);
		steering = mapPWM(steering, STEERING_DEADBAND);
		
		if (throttle == 0.0) {
			quickturn = true;
		} 
		
		if (quickturn) {
			angVelTeleopPID.disable();
			leftMotorSetter.disable();
			rightMotorSetter.disable();
			
			simpleDrive(0, steering);
		} else {
			

			leftMotorSetter.enable();
			rightMotorSetter.enable();
	
			
			
			double maxDriveCurrent = throttle * (getMaxCurrent() - Math.abs(curvatureTN.pidGet()));
			double velAvg = (RobotMap.driveLeftEncoderWrapperRate.pidGet() + RobotMap.driveRightEncoderWrapperRate.pidGet())/2;
			if (velAvg > 5) {
				angVelTeleopPID.enable();
				angVelTeleopPID.setSetpoint((2/RobotMap.ROBOT_WIDTH) * steering);
			} else {
				angVelTeleopPID.disable();
				angVelTeleopTN.setOutput(0);
			}
			velocityTN.setOutput(maxDriveCurrent);
			curvatureTN.setOutput(angVelTeleopTN.getOutput() * velAvg);
			
		}
		
		
		
	} */
	
	public double getAngleRateError() {
		return curvaturePID.getAvgError();
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
		return curvatureTN.getOutput();
	}

	public void setVelocities(double linearVel, double curvature) {
		
		velocityPID.enable();
		anglePID.disable();
		distancePID.disable();
		angVelTeleopPID.disable();
		leftMotorSetter.enable();
		rightMotorSetter.enable();

		// allows us to set setpoints directly
		velocityPID.setSetpointSource(null);
		curvaturePID.setSetpointSource(null);

		velocityPID.setSetpoint(linearVel);
		
		if (Math.abs(encoderAvgVelocityPIDSource.pidGet()) > LOW_ENC_RATE) {
			curvaturePID.enable();
			curvaturePID.setSetpoint(curvature);
		} else {
			curvaturePID.disable();
			curvatureTN.setOutput(0);
		}
		
	}
	
	public void WARlordsDrive(double throttle, double steering) {
		throttle = mapPWM(throttle, THROTTLE_DEADBAND);
		steering = mapPWM(steering, STEERING_DEADBAND);
		
		enablePID(false);
		
		double velAvg = (RobotMap.driveLeftEncoderWrapperRate.pidGet() + RobotMap.driveRightEncoderWrapperRate.pidGet())/2;
		if (velAvg > 5) {
			angVelTeleopPID.enable();
			angVelTeleopPID.setSetpoint((2/RobotMap.ROBOT_WIDTH) * steering);
		} else {
			angVelTeleopPID.disable();
			angVelTeleopTN.setOutput(0);
		}
		
		double left = throttle + angVelTeleopTN.getOutput();
		double right = throttle - angVelTeleopTN.getOutput();

		System.out.println(throttle);
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
	
	public void setCurrents(double l, double r) {
		RobotMap.driveLeftCurrent.set(l);
		RobotMap.driveRightCurrent.set(r);
	}

	public boolean driveTo(double distance, double maxSpeed, double angle, double curvature, double tolerance) {
		velocityPID.enable();
		anglePID.enable();
		distancePID.enable();
		angVelTeleopPID.disable();
		leftMotorSetter.enable();
		rightMotorSetter.enable();
		anglePID.setSetpoint(angle);
		distancePID.setSetpoint(distance);
		curvatureSetpointTN.setOutput(curvature);
		distancePID.setAbsoluteTolerance(tolerance);
		if (Math.abs(encoderAvgVelocityPIDSource.pidGet()) > LOW_ENC_RATE) {
			curvaturePID.enable();
		} else {
			curvaturePID.disable();
			curvatureTN.setOutput(0);
		}
		
		distancePID.setOutputRange(-maxSpeed, maxSpeed);
		anglePID.setOutputRange(-2 / RobotMap.ROBOT_WIDTH, 2 / RobotMap.ROBOT_WIDTH);
		
		return distancePID.isOnTarget() && Math.abs(encoderAvgVelocityPIDSource.pidGet()) < LOW_ENC_RATE;
	}
	
	public void updateConstants() {
		for (TalonSRX driveTalon : RobotMap.driveTalons) {
			driveTalon.enableVoltageCompensation(false);
//			driveTalon.configVoltageCompSaturation(ConstantsIO.voltageMax, 0);
			driveTalon.enableCurrentLimit(false);
//			driveTalon.configContinuousCurrentLimit(ConstantsIO.IMax, 0);
//			driveTalon.configOpenloopRamp(.2, 0);
		}
		velocityPID.setPID(ConstantsIO.kP_DriveVelocity, ConstantsIO.kI_DriveVelocity, ConstantsIO.kD_DriveVelocity, ConstantsIO.kF_DriveVelocity);
		curvaturePID.setPID(ConstantsIO.kP_DriveAngVel, ConstantsIO.kI_DriveAngVel, ConstantsIO.kD_DriveAngVel, ConstantsIO.kF_DriveAngVel);
		anglePID.setPID(ConstantsIO.kP_DriveAngle, ConstantsIO.kI_DriveAngle, ConstantsIO.kD_DriveAngle);
//		angVelTeleopPID.setPID(ConstantsIO.kP_AngVelTeleop, ConstantsIO.kI_AngVelTeleop, ConstantsIO.kD_AngVelTeleop, ConstantsIO.kF_AngVelTeleop);
		
	}

	public void enablePID(boolean enable) {
		if (enable) {
			velocityPID.enable();
			curvaturePID.enable();
			anglePID.enable();
			distancePID.enable();
		} else {
			velocityPID.disable();
			curvaturePID.disable();
			anglePID.disable();
			distancePID.disable();
			angVelTeleopPID.disable();
		}
	}
	
	public double getAverageEncoderDistance() {
		// TODO Auto-generated method stub
		return encoderDistancePIDSource.pidGet();
	}

	public double getTeleopAngVelError() {
		return angVelTeleopTN.getOutput();
	}
}
