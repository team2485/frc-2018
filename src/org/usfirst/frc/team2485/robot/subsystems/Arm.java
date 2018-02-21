package org.usfirst.frc.team2485.robot.subsystems;

import java.util.Timer;
import java.util.TimerTask;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.WristWithControllers;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.MotorSetter;
import org.usfirst.frc.team2485.util.PIDSourceWrapper;
import org.usfirst.frc.team2485.util.TransferNode;
import org.usfirst.frc.team2485.util.WarlordsPIDController;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Arm extends Subsystem {

	public static final double CRITICAL_DISTANCE = toMeters(35); // temp //distance from mast to 16 inches past frame
																	// perimeter
	public static final double ALPHA_MAX_WRIST = 8;
	public static final double ALPHA_MAX_ELBOW = 4;
	public static final int ELBOW_OFFSET = -4038;
	public static final int WRIST_OFFSET = -852;
	public static final double MIN_WRIST_LIFTING_POSITION = 0.1;

	private double thetaHigh;
	private double thetaLow;

	public double getThetaLow() {
		return thetaLow;
	}

	public void setThetaLow(double thetaLow) {
		this.thetaLow = thetaLow;
	}

	public double getThetaHigh() {
		return thetaHigh;
	}

	public void setThetaHigh(double thetaHigh) {
		this.thetaHigh = thetaHigh;
	}

	public static enum ArmSetpoint {
		INTAKE(-.2, 0), SWITCH(-.2, 0.15), SECOND_STORY(-.2, 0.065), SCALE_HIGH_BACK(0.16, 0.42), SCALE_MIDDLE_BACK(0.16, 0.46), SCALE2(0.22, 0), SCALE_HIGH(0.25, 0.125), SCALE_LOW_BACK(0.16, 0.5);

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
	public static final double MAX_CURRENT_WRIST = 20;
	public static final double MAX_CURRENT_ELBOW = 20;

	public static final double J1 = toMetricInertia(I1) + (m1 * d1 * d1);
	public static final double J2 = toMetricInertia(I2) + (m2 * d2 * d2);

	public static final double torqueStall = .71;
	public static final double gearRatioWrist = 975; // temporary
	public static final double gearRatioElbow = 945;

	public static final double CRITICAL_ANGLE = 0.15; // SET DIRECTLY
	private static final double WRIST_TOLERANCE = 0.0025;
	private static final double MAX_WRIST_ANGLE = .3;

	private WarlordsPIDController elbowAngPID = new WarlordsPIDController();
	private WarlordsPIDController elbowAngVelPID = new WarlordsPIDController();
	private WarlordsPIDController wristAngPID = new WarlordsPIDController();
	private WarlordsPIDController wristAngVelPID = new WarlordsPIDController();

	private TransferNode elbowAngTN = new TransferNode(0);
	private TransferNode elbowAngVelTN = new TransferNode(0);
	private TransferNode wristAngTN = new TransferNode(0);
	private TransferNode wristAngVelTN = new TransferNode(0);

	private PIDSourceWrapper kPElbowAngSource = new PIDSourceWrapper();
	private PIDSourceWrapper wristAngSource = new PIDSourceWrapper();
	private PIDSourceWrapper wristAngVelSource = new PIDSourceWrapper();
	private PIDSourceWrapper elbowCurrentSource = new PIDSourceWrapper();
	private PIDSourceWrapper wristCurrentSource = new PIDSourceWrapper();
	private PIDSourceWrapper elbowMinCurrentSource = new PIDSourceWrapper();
	private PIDSourceWrapper wristMaxCurrentSource = new PIDSourceWrapper();
	

	private MotorSetter elbowSetter = new MotorSetter();
	private MotorSetter wristSetter = new MotorSetter();

	public Arm() {

//		new Timer().schedule(new InitEncoderTask(), 0, 1000);
		// Elbow

		elbowAngPID.setSources(RobotMap.elbowEncoderWrapperDistance);
		elbowAngPID.setOutputs(elbowAngTN);
		elbowAngPID.setOutputRange(-.1, .25);
		elbowAngPID.setConstantsSources(kPElbowAngSource, null, null, null);

		elbowAngVelPID.setSources(RobotMap.elbowEncoderWrapperRate);
		elbowAngVelPID.setSetpointSource(elbowAngTN);
		elbowAngVelPID.setOutputs(elbowAngVelTN);
		elbowAngVelPID.setOutputRange(-MAX_CURRENT_ELBOW, MAX_CURRENT_ELBOW);
		elbowAngVelPID.setOutputSources(null, elbowMinCurrentSource);

		elbowMinCurrentSource.setPidSource(() -> {
			return RobotMap.elbowEncoderWrapperDistance.pidGet() < -CRITICAL_ANGLE ? -2 : -8;
		});
		
		wristMaxCurrentSource.setPidSource(() -> {
			if (RobotMap.wristEncoderWrapperRate.pidGet() == 0) {
				return 2;
			}
			return RobotMap.wristEncoderWrapperDistance.pidGet() > MAX_WRIST_ANGLE ? 1 : MAX_CURRENT_WRIST;
		});

		elbowCurrentSource.setPidSource(() -> {
			double a1 = elbowAngVelTN.pidGet();
			// double a2 = wristAngVelTN.pidGet();
			//
			// double theta1 = getElbowAngle();
			// double theta2 = getWristAngle();
			//
			// double w1 = 2*Math.PI*RobotMap.elbowEncoderWrapperRate.pidGet();
			// double w2 = 2*Math.PI*RobotMap.wristEncoderWrapperRate.pidGet() + w1;
			//
			// double inertia = (J1 * a1) +
			//// (getJ2() * a2) +
			// (L1*L1 * a1 * getM2()) +
			// (L1 * a2 * getD2() * getM2() * Math.cos(theta1 - theta2));
			// double load =
			// (L1 * g * getM2() * Math.cos(theta1)) +
			// (d1 * g * m1 * Math.cos(theta1)) +
			// (getD2() * g * getM2() * Math.cos(theta2));
			//// double linking = (L1 * getD2() * getM2() * w2*w2 * Math.sin(theta1 -
			// theta2));
			// double linking = 0;
			// double torque = inertia + load + linking;
			// double current = torque * (ConstantsIO.currentStallElbow / (2 *
			// gearRatioElbow * torqueStall));
			//
			// return Math.min(Math.max(current, -MAX_CURRENT_ELBOW), MAX_CURRENT_ELBOW);
			return a1
					+ ConstantsIO.levitateElbowCurrent
							* Math.cos(RobotMap.elbowEncoderWrapperDistance.pidGet() * 2 * Math.PI)
					+ ConstantsIO.levitateWristCurrent * Math.cos(wristAngSource.pidGet() * Math.PI * 2) / 5;
		});

		elbowSetter.setSetpointSource(elbowCurrentSource);
		elbowSetter.setOutputs(RobotMap.elbowCurrentWrapper);

		// Wrist
		
		kPElbowAngSource.setPidSource(() -> {
			return Math.min(ConstantsIO.kP_ElbowAng, ConstantsIO.accelerationMaxElbow/Math.abs(RobotMap.elbowEncoderWrapperRate.pidGet()));
		});

		wristAngSource.setPidSource(() -> {
			return RobotMap.elbowEncoderWrapperDistance.pidGet() + RobotMap.wristEncoderWrapperDistance.pidGet();
		});

		wristAngVelSource.setPidSource(() -> {
			return RobotMap.elbowEncoderWrapperRate.pidGet() + RobotMap.wristEncoderWrapperRate.pidGet();
		});

		wristAngPID.setSources(wristAngSource);
		wristAngPID.setOutputs(wristAngTN);
		wristAngPID.setOutputRange(-.3, .3);

		wristAngVelPID.setSources(wristAngVelSource);
		wristAngVelPID.setSetpointSource(wristAngTN);
		wristAngVelPID.setOutputs(wristAngVelTN);
		wristAngVelPID.setOutputRange(-MAX_CURRENT_WRIST, MAX_CURRENT_WRIST);

		wristCurrentSource.setPidSource(() -> {

			double a2 = wristAngVelTN.pidGet();

			// double theta2 = 2*Math.PI*RobotMap.wristEncoderWrapperDistance.pidGet();
			//
			// double inertia = (getJ2() * a2);
			// double load = (getD2() * g * getM2() * Math.cos(theta2));
			// double torque = inertia + load;
			// double current = torque * (ConstantsIO.currentStallWrist / (gearRatioWrist *
			// torqueStall));
			// return Math.min(Math.max(current, -MAX_CURRENT_WRIST), MAX_CURRENT_WRIST);
			return a2 + ConstantsIO.levitateWristCurrent * Math.cos(wristAngSource.pidGet() * Math.PI * 2);
		});

		wristSetter.setSetpointSource(wristCurrentSource);
		wristSetter.setOutputs(RobotMap.wristCurrentWrapper);

	}

	public void initDefaultCommand() {
		setDefaultCommand(new WristWithControllers());
	}
	
	public boolean isEncodersWorking() {
		boolean isEncoder = RobotMap.elbowTalon.getSensorCollection().getPulseWidthRiseToRiseUs() > 0 && RobotMap.wristTalon.getSensorCollection().getPulseWidthRiseToRiseUs() > 0;
		return isEncoder;
	}
	
	public void reset() {
//		RobotMap.arm.setElbowPos(ArmSetpoint.SWITCH.elbowPos);
//    	RobotMap.arm.setThetaLow(ArmSetpoint.SWITCH.wristPos);
		double theta2 = RobotMap.arm.getWristAngle();
    	if (RobotMap.arm.getElbowAngle() > 0) {
			RobotMap.arm.setThetaHigh(theta2);
		} else {
			RobotMap.arm.setThetaLow(theta2);
		}
		RobotMap.arm.setElbowPos(RobotMap.elbowEncoderWrapperDistance.pidGet());
    	elbowAngTN.setOutput(0);
    	wristAngTN.setOutput(0);
    	elbowAngVelTN.setOutput(0);
    	wristAngVelTN.setOutput(0);

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
		if (Math.abs(pos - wristAngSource.pidGet()) < WRIST_TOLERANCE) {
			setWristManual(0);
		} else {
			wristAngPID.enable();
			wristAngVelPID.enable();
			wristSetter.enable();
		}
		// System.out.println(pos);
	}

	public void setWristManual(double pwm) {
		wristAngPID.disable();
		wristAngVelPID.disable();
		wristAngVelTN.setOutput(pwm * ALPHA_MAX_WRIST);
		wristSetter.enable();
	}

	public void setElbowManual(double pwm) {
		elbowAngPID.disable();
		elbowAngVelPID.disable();
		elbowAngVelTN.setOutput(pwm * ALPHA_MAX_ELBOW);
		elbowSetter.enable();
	}

	public double getMinWristPos() {
		double margin = (CRITICAL_DISTANCE - (L1 * Math.cos(getElbowAngle() * 2 * Math.PI))) / L2;
		return Math.abs(RobotMap.elbowEncoderWrapperDistance.pidGet()) < CRITICAL_ANGLE
				? Math.acos(margin) / 2 / Math.PI
				: -0.25;
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

	public void setElbowPos(double pos) {
		elbowAngPID.enable();
		elbowAngVelPID.enable();
		elbowSetter.enable();
		elbowAngPID.setSetpoint(pos);
	}

	public double getElbowAngle() {
		return RobotMap.elbowEncoderWrapperDistance.pidGet();
	}

	public double getWristAngle() {
		return (RobotMap.wristEncoderWrapperDistance.pidGet()) + getElbowAngle();
	}

	// Testing
	public void setElbowCurrent(double current) {
		if (elbowPIDisEnabled()) {
			elbowAngPID.disable();
			elbowAngVelPID.disable();
			elbowAngTN.setOutput(0);
			elbowAngVelTN.setOutput(0);
		}
		if (wristPIDisEnabled()) {
			wristAngPID.disable();
			wristAngVelPID.disable();
			wristAngTN.setOutput(0);
			wristAngVelTN.setOutput(0);
		}

		elbowSetter.disable();
		RobotMap.elbowCurrentWrapper.set(current);
	}

	public void setWristCurrent(double current) {
		if (elbowPIDisEnabled()) {
			elbowAngPID.disable();
			elbowAngVelPID.disable();
			elbowAngTN.setOutput(0);
			elbowAngVelTN.setOutput(0);
		}
		if (wristPIDisEnabled()) {
			wristAngPID.disable();
			wristAngVelPID.disable();
			wristAngTN.setOutput(0);
			wristAngVelTN.setOutput(0);
		}

		// wristSetter.disable();
		RobotMap.wristCurrentWrapper.set(current);
	}

	public void setElbowVelocity(double angVel) {
		if (!elbowPIDisEnabled()) {
			elbowAngVelPID.enable();
			elbowAngPID.disable();
		}

		elbowSetter.enable();
		elbowAngTN.setOutput(angVel);
	}

	public void setWristVelocity(double angVel) {
		if (!wristAngVelPID.isEnabled()) {
			wristAngVelPID.enable();
			wristAngPID.disable();
		}
		wristSetter.enable();

		wristAngTN.setOutput(angVel);
	}

	public double getWristAngVelError() {
		return wristAngVelPID.getAvgError();
	}

	public double getWristAngError() {
		return wristAngPID.getAvgError();
	}

	public double getElbowAngVelError() {
		return elbowAngVelPID.getAvgError();
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

	public boolean wristPIDisEnabled() {
		return wristAngVelPID.isEnabled(); // wristAngPID.isEnabled() &&
	}

	public boolean elbowPIDisEnabled() {
		return elbowAngPID.isEnabled() && elbowAngVelPID.isEnabled();
	}

	public void updateConstants() {
		elbowAngVelPID.setPID(ConstantsIO.kP_ElbowAngVel, ConstantsIO.kI_ElbowAngVel, 0, ConstantsIO.kF_ElbowAngVel);
		wristAngPID.setPID(ConstantsIO.kP_WristAng, 0, 0);
		wristAngVelPID.setPID(ConstantsIO.kP_WristAngVel, ConstantsIO.kI_WristAngVel, 0, ConstantsIO.kF_WristAngVel);
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

	public class InitEncoderTask extends TimerTask {

		@Override
		public void run() {
//			if(isEncodersWorking()) {
//				initElbowEnc();
//				initWristEnc();
//			}
		}
		
	}
}
