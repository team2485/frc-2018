package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.DriveWithControllers;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.MotorSetter;
import org.usfirst.frc.team2485.util.PIDSourceWrapper;
import org.usfirst.frc.team2485.util.PigeonWrapperRateAndAngle;
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


	private WarlordsPIDController distancePID = new WarlordsPIDController();
	private WarlordsPIDController anglePID = new WarlordsPIDController();
	private WarlordsPIDController velocityPID = new WarlordsPIDController();
	private WarlordsPIDController angVelocityPID = new WarlordsPIDController();

	private TransferNode distanceTN = new TransferNode(0);
	private TransferNode angleTN = new TransferNode(0);
	public TransferNode velocityTN = new TransferNode(0);
	public TransferNode angVelocityTN = new TransferNode(0);

	private TransferNode curvatureTN = new TransferNode(0);

	private PIDSourceWrapper kp_distancePIDSource = new PIDSourceWrapper();

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
		double Vl1 = RobotMap.driveLeftTalon1.getMotorOutputVoltage();
		double Vl2 = RobotMap.driveLeftTalon2.getMotorOutputVoltage();
		double Vl3 = RobotMap.driveLeftTalon3.getMotorOutputVoltage();
		double VlAvg = Math.abs((Vl1+Vl2+Vl3)/3);

		double Vr1 = RobotMap.driveRightTalon1.getMotorOutputVoltage();
		double Vr2 = RobotMap.driveRightTalon2.getMotorOutputVoltage();
		double Vr3 = RobotMap.driveRightTalon3.getMotorOutputVoltage();
		double VrAvg = Math.abs((Vr1+Vr2+Vr3)/3);

		double iMax = ConstantsIO.IMax;

		double Ir1 = RobotMap.driveRightTalon1.getOutputCurrent();
		double Ir2 = RobotMap.driveRightTalon2.getOutputCurrent();
		double Ir3 = RobotMap.driveRightTalon3.getOutputCurrent();
		double iL = (Ir1+Ir2+Ir3)/3;

		double Il1 = RobotMap.driveLeftTalon1.getOutputCurrent();
		double Il2 = RobotMap.driveLeftTalon2.getOutputCurrent();
		double Il3 = RobotMap.driveLeftTalon3.getOutputCurrent();
		double iR = (Il1+Il2+Il3)/3;

		double v = vBat;

		if(iL>iMax/2) {
			v=Math.min(v,VlAvg*iMax/iL);
		}
		if(iR >iMax/2) {
			v=Math.min(v,VrAvg*iMax/iR);
		}
		return v;
	}
	
	public double getMaxCurrent() {
		double pwmL1 = RobotMap.driveLeftTalon1.getMotorOutputPercent();
		double pwmL2 = RobotMap.driveLeftTalon2.getMotorOutputPercent();
		double pwmL3 = RobotMap.driveLeftTalon3.getMotorOutputPercent();
		double pwmL = (pwmL1 + pwmL2 + pwmL3)/3;
		
		double pwmR1 = RobotMap.driveRightTalon1.getMotorOutputPercent();
		double pwmR2 = RobotMap.driveRightTalon2.getMotorOutputPercent();
		double pwmR3 = RobotMap.driveRightTalon3.getMotorOutputPercent();
		double pwmR = (pwmR1 + pwmR2 + pwmR3)/3;

		double Ir1 = RobotMap.driveRightTalon1.getOutputCurrent();
		double Ir2 = RobotMap.driveRightTalon2.getOutputCurrent();
		double Ir3 = RobotMap.driveRightTalon3.getOutputCurrent();
		double iL = (Ir1+Ir2+Ir3)/3;

		double Il1 = RobotMap.driveLeftTalon1.getOutputCurrent();
		double Il2 = RobotMap.driveLeftTalon2.getOutputCurrent();
		double Il3 = RobotMap.driveLeftTalon3.getOutputCurrent();
		double iR = (Il1+Il2+Il3)/3;
		
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
			return Math.min(ConstantsIO.kPMax_Distance, 2*ConstantsIO.accelerationMax/encoderAvgVelocityPIDSource.pidGet());
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
			return -(getMaxVoltage()/ConstantsIO.voltageMax - Math.abs(angVelocityTN.pidGet()));
		});
		maxVelocityORSource.setPidSource(() -> {
			return getMaxVoltage()/ConstantsIO.voltageMax - Math.abs(angVelocityTN.pidGet());

		});
		
		velocityPID.setOutputs(velocityTN);
		velocityPID.setSources(encoderAvgVelocityPIDSource);
		velocityPID.setSetpointSource(distanceTN);
//		velocityPID.setOutputSources(maxVelocityORSource, minVelocityORSource);
		

		//angle

		anglePID.setOutputs(angleTN);
		anglePID.setSources(RobotMap.pigeonDisplacementWrapper);
		anglePID.setContinuous(true);
		anglePID.setInputRange(0, 2 * Math.PI);

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
		leftMotorSetter.setOutputs(RobotMap.driveLeft);
		rightMotorSetter.setSetpointSource(rightCurrentPIDSource);
		rightMotorSetter.setOutputs(RobotMap.driveRight);

	}

	public void initDefaultCommand() {
//		setDefaultCommand(new DriveWithControllers());
	}

	public void simpleDrive(double throttle, double steering) {
		velocityPID.disable();
		angVelocityPID.disable();
		rightMotorSetter.disable();
		leftMotorSetter.disable();
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

		RobotMap.driveLeft.set(left);
		RobotMap.driveRight.set(right);
	}

	public void WARLordsDrive(double throttle, double steering) {


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

		RobotMap.driveLeft.set(0);
		RobotMap.driveRight.set(0);
		velocityPID.disable();
		angVelocityPID.disable();
		rightMotorSetter.disable();
		leftMotorSetter.disable();
		

	}
	
	public void emergencyStop() {
		reset();
	}

	public void setVelocities(double linearVel, double angVel) {
		velocityPID.enable();
//		angVelocityPID.enable();
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
	

	public void driveTo(double distance, double maxSpeed, double angle, double curvature) {
		velocityPID.enable();
//		angVelocityPID.enable();
//		anglePID.enable();
		distancePID.enable();
		leftMotorSetter.enable();
		rightMotorSetter.enable();
//		anglePID.setSetpoint(angle);
		distancePID.setSetpoint(distance);
//		curvatureTN.setOutput(curvature);
		
		distancePID.setOutputRange(-maxSpeed, maxSpeed);
//		anglePID.setOutputRange(-maxSpeed / RobotMap.ROBOT_WIDTH, maxSpeed / RobotMap.ROBOT_WIDTH);
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
		angVelocityPID.setPID(ConstantsIO.kP_DriveAngVel, ConstantsIO.kI_DriveAngVel, ConstantsIO.kD_DriveAngVel, ConstantsIO.kF_DriveAngVel);
		anglePID.setPID(ConstantsIO.kP_DriveAngle, ConstantsIO.kI_DriveAngle, ConstantsIO.kD_DriveAngle);

	}
}
