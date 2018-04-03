package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.WristWithControllers;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.FastMath;
import org.usfirst.frc.team2485.util.PIDOutputWrapper;
import org.usfirst.frc.team2485.util.PIDSourceWrapper;
import org.usfirst.frc.team2485.util.TransferNode;
import org.usfirst.frc.team2485.util.WarlordsPIDController;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Arm extends Subsystem {

	
	public static final double CRITICAL_DISTANCE = toMeters(40); // temp //distance from mast to 16 inches past frame
																	// perimeter
	public static final double CONSERVATIVE_CRITICAL_DISTANCE = toMeters(32);
	public static final double ALPHA_MAX_WRIST = 8;
	public static final double ALPHA_MAX_ELBOW = 4;
	public static final int ELBOW_OFFSET = -4038;
	public static final int WRIST_OFFSET = -852;
	public static final double MIN_WRIST_LIFTING_POSITION = 0.1;

	private double thetaWrist;
	private double thetaElbow;


	public void setThetaWrist(double thetaWrist) {
		this.thetaWrist = thetaWrist;
		WristWithControllers.manualSetpoint = false;
	}
	
	public double getThetaWrist() {
		return thetaWrist;
	}

	

	public static enum ArmSetpoint {
		INTAKE(-.192, -0.030), SWITCH(-.192, 0.15), SECOND_STORY(-.192, 0.025), SCALE_HIGH_BACK(0.22, 0.4), SCALE_MIDDLE_BACK(0.2, 0.46), SCALE_LOW_BACK(0.16, 0.52), SEVEN_FOOT_SCALE(.22, .32), CLIMB(.26, .225);

		private final double elbowPos;
		private final double wristPos;

		ArmSetpoint(double elbowPos, double wristPos) {
			this.elbowPos = elbowPos;
			this.wristPos = wristPos;
		}

		public double getElbowPos() {
			return this.elbowPos;
		}

		public double getWristPos() {
			return this.wristPos;
		}

	}

	/*/@\
	 * 
	 * arm = 1 hand = 2 L = length d = center of mass m = mass j = moment of inertia
	 */

	public static final double L1 = toMeters(30.83);
	public static final double L2 = toMeters(29.534);
	public static final double LBox = toMeters(13);
	public static final double d1 = toMeters(14.5);
	public static final double d2 = toMeters(20.44);
	public static final double m1 = toKilograms(3.55);
	public static final double m2 = toKilograms(12.5);
	public static final double mBox = toKilograms(3.5);
	public static final double g = 9.80665;
	public static final double I1 = 386.33;
	public static final double I2 = 499;
	

	public static final double J1 = toMetricInertia(I1) + (m1 * d1 * d1);
	public static final double J2 = toMetricInertia(I2) + (m2 * d2 * d2);

	public static final double torqueStall = .71;
	public static final double gearRatioWrist = 975; // temporary
	public static final double gearRatioElbow = 945;

	public static final double CRITICAL_ANGLE = 0.15; // SET DIRECTLY
	public static final double WRIST_TOLERANCE = 0;
	public static final double ELBOW_TOLERANCE = 0;

	
	// encoder failsafes
	private static final double MIN_WRIST_ANGLE = -0.35;
	private static final double MAX_WRIST_ANGLE = .4;
	private static final double MIN_ELBOW_ANGLE = -.14;
	private static final double MAX_ELBOW_ANGLE = 0.25;
	private static final double LOW_ENC_RATE = 0.0025;
	private static final double MAX_PWM_THRESHOLD_NO_CURRENT = .1;
	public static final double MAX_UP_CURRENT_WRIST = 20;
	public static final double MAX_DOWN_CURRENT_WRIST = 20;
	public static final double MAX_UP_CURRENT_ELBOW = 25;
	public static final double MAX_DOWN_CURRENT_ELBOW = 20;
	
	public static final double VMAX_WRIST = .2; //.4
	public static final double VMIN_WRIST = -.2; //-.4
	public static final double VMAX_ELBOW = .1;
	public static final double VMIN_ELBOW = -.07;
	
	public static final double QUASI_LOW_ELBOW_RATE = .01;
	
	public static double MIN_WRIST_ANGLE_CROSS;
	public static double MID_ELBOW_ANGLE;
	
	public boolean isClimb = false;

	private WarlordsPIDController elbowAngPID = new WarlordsPIDController();
	public WarlordsPIDController elbowAngVelMaxPID = new WarlordsPIDController();
	public WarlordsPIDController elbowAngVelMinPID = new WarlordsPIDController();
	public WarlordsPIDController wristAngPID = new WarlordsPIDController();
//	public  WarlordsPIDController wristAngVelMaxPID = new WarlordsPIDController();
//	public WarlordsPIDController wristAngVelMinPID = new WarlordsPIDController();

	public TransferNode wristAngVelMaxTN = new TransferNode(0);
	public TransferNode wristAngVelMinTN = new TransferNode(0);
	public TransferNode elbowAngVelMaxTN = new TransferNode(0);
	public TransferNode elbowAngVelMinTN = new TransferNode(0);

	private PIDSourceWrapper wristAngSource = new PIDSourceWrapper();
	private PIDSourceWrapper wristAngVelSource = new PIDSourceWrapper();
	public PIDSourceWrapper wristMinCurrentSource = new PIDSourceWrapper();
	public PIDSourceWrapper wristMaxCurrentSource = new PIDSourceWrapper();
	public PIDSourceWrapper elbowMinCurrentSource = new PIDSourceWrapper();
	public PIDSourceWrapper elbowMaxCurrentSource = new PIDSourceWrapper();
	public PIDSourceWrapper elbowMinAngVelSource = new PIDSourceWrapper();
	public PIDSourceWrapper elbowMaxAngVelSource = new PIDSourceWrapper();
	public PIDSourceWrapper wristMaxAngSource = new PIDSourceWrapper();
	public PIDSourceWrapper wristMinAngSource = new PIDSourceWrapper();
	public PIDSourceWrapper elbowMaxAngSource = new PIDSourceWrapper();
	public PIDSourceWrapper elbowMinAngSource = new PIDSourceWrapper();
	public PIDOutputWrapper elbowOutputSource = new PIDOutputWrapper();
	public PIDOutputWrapper wristOutputSource = new PIDOutputWrapper();

	
	
	public double[] getThetasCritical() {
		double theta2 = RobotMap.wristEncoderWrapperDistance.pidGet() * 2 * Math.PI;
		double x = L1 + L2 * FastMath.cos(theta2);
		double y = L2 * FastMath.sin(theta2);
		double angle = FastMath.atan2(y, x) / 2 / Math.PI;
		if (angle > .5) {
			angle -= 1;
		}
		double dist = FastMath.hypot(x, y);
		double margin = CRITICAL_DISTANCE / dist;
		if (margin > 1) { 
			return new double[] {0, 0};
		}
		double criticalAngle = FastMath.acos(margin) / Math.PI / 2;
		return new double[] {-angle, criticalAngle};
	}
	

	public double getIMaxElbow() {
		if (Math.abs(RobotMap.elbowEncoderWrapperRate.pidGet()) < LOW_ENC_RATE) {
			return 3;
		}
		double i = RobotMap.elbowTalon.getOutputCurrent();
		double pwm = RobotMap.elbowTalon.getMotorOutputPercent();
		double iMax = MAX_UP_CURRENT_ELBOW;
		if (Math.abs(pwm) > .5) {
			iMax = Math.min(iMax, Math.abs(i/pwm));
		}
		return RobotMap.elbowEncoderWrapperDistance.pidGet() > MAX_ELBOW_ANGLE ? 2 : iMax;
	}
	
	public double getIMinElbow() {
		if (Math.abs(RobotMap.elbowEncoderWrapperRate.pidGet()) < LOW_ENC_RATE) {
			return -2;
		}
		if (RobotMap.wristEncoderWrapperDistance.pidGet() > MAX_WRIST_ANGLE) { // if wrist at hard stop don't move arm down
			return -2;
		}
		double i = RobotMap.elbowTalon.getOutputCurrent();
		double pwm = RobotMap.elbowTalon.getMotorOutputPercent();
		double iMin = MAX_DOWN_CURRENT_ELBOW;
		if (Math.abs(pwm) > .5) {
			iMin = Math.min(iMin, Math.abs(i/pwm));
		}
		return RobotMap.elbowEncoderWrapperDistance.pidGet() < MIN_ELBOW_ANGLE ? -2 : -iMin;
	}
	
	public double getIMaxWrist() {
		if (Math.abs(RobotMap.wristEncoderWrapperRate.pidGet()) < LOW_ENC_RATE && Math.abs(RobotMap.elbowEncoderWrapperRate.pidGet()) < QUASI_LOW_ELBOW_RATE) {
			return 4;
		}
		double i = RobotMap.wristTalon.getOutputCurrent();
		double pwm = RobotMap.wristTalon.getMotorOutputPercent();
		double iMax = MAX_UP_CURRENT_WRIST;
		if (Math.abs(pwm) > .5) {
			iMax = Math.min(iMax, Math.abs(i/pwm));
		}
		return RobotMap.wristEncoderWrapperDistance.pidGet() > MAX_WRIST_ANGLE ? 2 : iMax;
	}
	
	public double getIMinWrist() {
		if (Math.abs(RobotMap.wristEncoderWrapperRate.pidGet()) < LOW_ENC_RATE && Math.abs(RobotMap.elbowEncoderWrapperRate.pidGet()) < QUASI_LOW_ELBOW_RATE) {
			return -4;
		}
		double i = RobotMap.wristTalon.getOutputCurrent();
		double pwm = RobotMap.wristTalon.getMotorOutputPercent();
		double iMin = MAX_DOWN_CURRENT_WRIST;
		if (Math.abs(pwm) > .5) {
			iMin = Math.min(iMin, Math.abs(i/pwm));
		}
		return RobotMap.wristEncoderWrapperDistance.pidGet() < MIN_WRIST_ANGLE ? -2 : -iMin;

	}
	

	public Arm() {
		
		MIN_WRIST_ANGLE_CROSS = FastMath.acos((CONSERVATIVE_CRITICAL_DISTANCE * CONSERVATIVE_CRITICAL_DISTANCE - L2 * L2 - L1 * L1) 
				/ 2 / L1 / L2) / 2 / Math.PI; 
		MID_ELBOW_ANGLE = -FastMath.acos((L1 * L1 + CONSERVATIVE_CRITICAL_DISTANCE * CONSERVATIVE_CRITICAL_DISTANCE - L2 * L2) 
				/ 2 / CONSERVATIVE_CRITICAL_DISTANCE / L1) / 2 / Math.PI;

		// Elbow
		elbowMaxAngSource.setPidSource(() -> {
			return elbowAngVelMaxTN.pidGet();
		});
		
		elbowMinAngSource.setPidSource(() -> {
			return elbowAngVelMinTN.pidGet();
		});
		
		elbowMinAngVelSource.setPidSource(() -> {
			return isClimb ? -.15 : -.3;
		});
		
		elbowMaxAngVelSource.setPidSource(() -> {
			return isClimb ? .15 : .3;
		});
		
		elbowMaxCurrentSource.setPidSource(() -> {
			return getIMaxElbow();
		});

		elbowMinCurrentSource.setPidSource(() -> {
			return getIMinElbow();
		});
		
		
		elbowAngVelMaxPID.setSources(RobotMap.elbowEncoderWrapperRate);
		elbowAngVelMaxPID.setOutputRange(0, 1);
		elbowAngVelMaxPID.setOutputSources(elbowMaxCurrentSource, null);
		elbowAngVelMaxPID.setOutputs(elbowAngVelMaxTN);
		elbowAngVelMaxPID.setSetpointSource(elbowMaxAngVelSource);
		elbowAngVelMaxPID.setPeriod(100);
		
		elbowAngVelMinPID.setSources(RobotMap.elbowEncoderWrapperRate);
		elbowAngVelMinPID.setOutputRange(-1, 0);
		elbowAngVelMinPID.setOutputSources(null, elbowMinCurrentSource);
		elbowAngVelMinPID.setOutputs(elbowAngVelMinTN);
		elbowAngVelMinPID.setSetpointSource(elbowMinAngVelSource);
		elbowAngVelMinPID.setPeriod(100);

		elbowAngPID.setSources(RobotMap.elbowEncoderWrapperDistance);
		elbowAngPID.setOutputSources(elbowMaxAngSource, elbowMinAngSource);
		elbowAngPID.setOutputs(elbowOutputSource);
		elbowAngPID.setVelocitySource(RobotMap.elbowEncoderWrapperRate);
		elbowAngPID.setPeriod(100);
		
		elbowOutputSource.setPidOutput((double out) -> {
			double pwm = out + ConstantsIO.levitateElbowCurrent * FastMath.cos(2 * Math.PI * getElbowAngle());
			RobotMap.elbowCurrentWrapper.set(pwm);
		});
		
		//Wrist
		
		wristMaxCurrentSource.setPidSource(() -> {
			return getIMaxWrist();
		});
		
		wristMinCurrentSource.setPidSource(() -> {
			return getIMinWrist();
		});
		
		wristAngVelSource.setPidSource(() -> {
			return RobotMap.wristEncoderWrapperRate.pidGet() + RobotMap.elbowEncoderWrapperRate.pidGet();
		});
		
//		wristAngVelMaxPID.setSources(RobotMap.wristEncoderWrapperRate);
//		wristAngVelMaxPID.setOutputRange(0, 1);
//		wristAngVelMaxPID.setOutputSources(wristMaxCurrentSource, null);
//		wristAngVelMaxPID.setSetpoint(VMAX_WRIST);
//		wristAngVelMaxPID.setOutputs(wristAngVelMaxTN);
//		
//		wristAngVelMinPID.setSources(RobotMap.wristEncoderWrapperRate);
//		wristAngVelMinPID.setSetpoint(VMIN_WRIST);
//		wristAngVelMinPID.setOutputRange(-1, 0);
//		wristAngVelMinPID.setOutputSources(null, wristMinCurrentSource);
//		wristAngVelMinPID.setOutputs(wristOutputSource);
//		
		wristAngSource.setPidSource(() -> {
			return RobotMap.wristEncoderWrapperDistance.pidGet() + RobotMap.elbowEncoderWrapperDistance.pidGet();
		});
		
		wristAngPID.setSources(RobotMap.wristEncoderWrapperDistance);
		wristAngPID.setOutputSources(wristMaxCurrentSource, wristMinCurrentSource);
		wristAngPID.setOutputs(wristOutputSource);
		wristAngPID.setVelocitySource(RobotMap.wristEncoderWrapperRate);
		wristAngPID.setPeriod(100);
		
		wristOutputSource.setPidOutput((double out) -> {
			double pwm = out; //+ ConstantsIO.levitateWristCurrent * FastMath.cos(2 * Math.PI * getWristAngle());
			RobotMap.wristCurrentWrapper.set(pwm);
		});

	}

	public void initDefaultCommand() {
		setDefaultCommand(new WristWithControllers());
	}
	
	public boolean isElbowCurrentSensorWorking() {
		boolean isElbowTalonWorking = RobotMap.elbowTalon.getOutputCurrent() == 0 && RobotMap.elbowTalon.getMotorOutputPercent() > MAX_PWM_THRESHOLD_NO_CURRENT;
		return !isElbowTalonWorking;
	}
	
	public boolean isWristCurrentSensorWorking() {
		boolean isWristTalonWorking = RobotMap.wristTalon.getOutputCurrent() == 0 && RobotMap.wristTalon.getMotorOutputPercent() > MAX_PWM_THRESHOLD_NO_CURRENT;
		return !isWristTalonWorking;
	}
	
	public void reset() {
//		RobotMap.arm.setElbowPos(ArmSetpoint.SWITCH.elbowPos);
//    	RobotMap.arm.setThetaLow(ArmSetpoint.SWITCH.wristPos);
		RobotMap.arm.setThetaWrist(RobotMap.wristEncoderWrapperDistance.pidGet());
		RobotMap.arm.setThetaElbow(RobotMap.elbowEncoderWrapperDistance.pidGet());
    	RobotMap.elbowCurrentWrapper.set(0);
    	RobotMap.wristCurrentWrapper.set(0);

	}
	
	public void setIsClimb(boolean isClimb) {
		this.isClimb = isClimb;
		System.out.println("set climb:"+ isClimb);
	}
	
	public void setElbowSetpoint(double setpoint) {
		if (Math.abs(setpoint - getElbowAngle()) < ELBOW_TOLERANCE) {
			setElbowManual(0);
		} else {
			elbowAngPID.enable();
			elbowAngVelMaxPID.enable();
			elbowAngVelMinPID.enable();
			elbowAngPID.setSetpoint(setpoint);
		}
	}

	public double getM2() {
		return RobotMap.intake.hasCube() ? m2 + mBox : m2;
	}

	public double getD2() {
		double d2Box = (d2 * m2 + LBox * mBox) / (m2 + mBox);
		return RobotMap.intake.hasCube() ? d2Box : d2;
	}

	public double getJ2() {
		return RobotMap.intake.hasCube() ? J2 + mBox * LBox * LBox : J2;
	}

	public void setWristPos(double pos) {
		wristAngPID.setSetpoint(pos);
//		wristAngVelMaxPID.setSetpoint(VMAX_WRIST);
//		wristAngVelMinPID.setSetpoint(VMIN_WRIST);
		if (Math.abs(pos - wristAngSource.pidGet()) < WRIST_TOLERANCE) {
			setWristManual(0);
		} else {
			wristAngPID.enable();
//			wristAngVelMaxPID.enable();
//			wristAngVelMinPID.enable();
		}
	}

	public void setWristManual(double pwm) {
		wristAngPID.disable();
//		wristAngVelMaxPID.disable();
//		wristAngVelMinPID.disable();
		wristOutputSource.pidWrite(pwm * ALPHA_MAX_WRIST);
	}

	public void setElbowManual(double pwm) {
		elbowAngPID.disable();
		elbowAngVelMaxPID.disable();
		elbowAngVelMinPID.disable();
		elbowOutputSource.pidWrite(pwm * ALPHA_MAX_ELBOW);
	}

	public double getMinWristPos() {
		double margin = (CONSERVATIVE_CRITICAL_DISTANCE - (L1 * FastMath.cos(getElbowAngle() * 2 * Math.PI))) / L2;
		return margin > 1 || Math.abs(getElbowAngle()) > CRITICAL_ANGLE ? -100 : FastMath.acos(margin) / 2 / Math.PI - RobotMap.elbowEncoderWrapperDistance.pidGet();
	}

	public void initElbowEnc() {
		int currPos = ELBOW_OFFSET + RobotMap.elbowTalon.getSensorCollection().getPulseWidthPosition();
		while (Math.abs(currPos) > 2048) {
			if (currPos > 0) {
				currPos -= 4096;
			} else {
				currPos += 4096;
			}
		}
		RobotMap.elbowEncoderWrapperDistance.setPosition(currPos);;
	}

	public void initWristEnc() {
		int currPos = WRIST_OFFSET + RobotMap.wristTalon.getSensorCollection().getPulseWidthPosition();
		while (Math.abs(currPos) > 2048) {
			if (currPos > 0) {
				currPos -= 4096;
			} else {
				currPos += 4096;
			}
		}
		RobotMap.wristEncoderWrapperDistance.setPosition(currPos);
	}

	public void setThetaElbow(double pos) {
		this.thetaElbow = pos;
	}
	
	public double getThetaElbow() {
		return thetaElbow;
	}

	public double getElbowAngle() {
		return RobotMap.elbowEncoderWrapperDistance.pidGet();
	}

	public double getWristAngle() {
		return (RobotMap.wristEncoderWrapperDistance.pidGet());
	}

	// Testing
	public void setElbowCurrent(double current) {
		if (elbowPIDisEnabled()) {
			elbowAngPID.disable();
			elbowAngVelMaxPID.disable();
			elbowAngVelMinPID.disable();
			RobotMap.elbowCurrentWrapper.set(0);
		}
		if (wristPIDisEnabled()) {
			wristAngPID.disable();
//			wristAngVelMaxPID.disable();
//			wristAngVelMinPID.disable();
			RobotMap.wristCurrentWrapper.set(0);
		}

		RobotMap.elbowCurrentWrapper.set(current);
	}

	public void setWristCurrent(double current) {
		if (elbowPIDisEnabled()) {
			elbowAngPID.disable();
			elbowAngVelMaxPID.disable();
			elbowAngVelMinPID.disable();
			RobotMap.elbowCurrentWrapper.set(0);
		}
		if (wristPIDisEnabled()) {
			wristAngPID.disable();
//			wristAngVelMaxPID.disable();
			elbowAngVelMinPID.disable();
			elbowAngVelMaxPID.disable();
			RobotMap.wristCurrentWrapper.set(0);
		}

		// wristSetter.disable();
		RobotMap.wristCurrentWrapper.set(current);
	}


//	public double getWristAngVelMaxError() {
//		return wristAngVelMaxPID.getAvgError();
//	}
//	
//	public double getWristAngVelMinError() {
//		return wristAngVelMinPID.getAvgError();
//	}

	public double getWristAngError() {
		return wristAngPID.getAvgError();
	}

	public double getElbowAngVelMaxError() {
		return elbowAngVelMaxPID.getAvgError();
	}
	
	public double getElbowAngVelMinError() {
		return elbowAngVelMinPID.getAvgError();
	}

	public double getElbowAngError() {
		return elbowAngPID.getAvgError();
	}

	public double getElbowCurrent() {
		return RobotMap.elbowTalon.getOutputCurrent();
	}

	public double getWristCurrent() {
		return RobotMap.wristTalon.getOutputCurrent();
	}

	public double getElbowCurrentError() {
		return RobotMap.elbowTalon.getClosedLoopError(0);
	}

	public double getWristCurrentError() {
		return RobotMap.wristTalon.getClosedLoopError(0);
	}
//
	public boolean wristPIDisEnabled() {
		return wristAngPID.isEnabled();
	}

	public boolean elbowPIDisEnabled() {
		return elbowAngPID.isEnabled() && elbowAngVelMaxPID.isEnabled();
	}

	public void updateConstants() {
		elbowAngPID.setPID(ConstantsIO.kP_ElbowAng, ConstantsIO.kI_ElbowAng, ConstantsIO.kD_ElbowAng);
		elbowAngVelMaxPID.setPID(ConstantsIO.kP_ElbowAngVel, ConstantsIO.kI_ElbowAngVel, 0, ConstantsIO.kF_ElbowAngVel);
		elbowAngVelMinPID.setPID(ConstantsIO.kP_ElbowAngVel, ConstantsIO.kI_ElbowAngVel, 0, ConstantsIO.kF_ElbowAngVel);
		wristAngPID.setPID(ConstantsIO.kP_WristAng, ConstantsIO.kI_WristAng, ConstantsIO.kD_WristAng);
//		wristAngVelMaxPID.setPID(ConstantsIO.kP_WristAngVel, ConstantsIO.kI_WristAngVel, 0, ConstantsIO.kF_WristAngVel);
//		wristAngVelMinPID.setPID(ConstantsIO.kP_WristAngVel, ConstantsIO.kI_WristAngVel, 0, ConstantsIO.kF_WristAngVel);
		RobotMap.elbowTalon.configForwardSoftLimitThreshold(ConstantsIO.kSoftLimitForward_Elbow, 0);
		RobotMap.elbowTalon.configForwardSoftLimitEnable(false, 0);
		RobotMap.elbowTalon.configReverseSoftLimitThreshold(ConstantsIO.kSoftLimitReverse_Elbow, 0);
		RobotMap.elbowTalon.configReverseSoftLimitEnable(false, 0);

	}

	public static double toMeters(double in) {
		return in * 0.0254;
	}

	public static double toKilograms(double lb) {
		return lb * 0.45359237;
	}

	public static double toMetricInertia(double lbin) {
		return toKilograms(toMeters(toMeters(lbin)));
	}

	public double getElbowSetpoint() {
		return elbowAngPID.getSetpoint();
	}

	
}
