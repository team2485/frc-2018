package org.usfirst.frc.team2485.robot.subsystems;

import java.util.Timer;
import java.util.TimerTask;

import org.usfirst.frc.team2485.robot.Robot;
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

	
	private int numBadCurrents = 0;
	public static final double CRITICAL_DISTANCE = toMeters(40); // temp //distance from mast to 16 inches past frame
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
		INTAKE(-.205, -0.015), SWITCH(-.205, 0.15), SECOND_STORY(-.205, 0.04), SCALE_HIGH_BACK(0.16, 0.39), SCALE_MIDDLE_BACK(0.16, 0.46), SCALE2(0.22, 0), SCALE_HIGH(0.25, 0.125), SCALE_LOW_BACK(0.16, 0.5), SEVEN_FOOT_SCALE(.26, .124);

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

	public static final double CRITICAL_ANGLE = 0.12; // SET DIRECTLY
	public static final double WRIST_TOLERANCE = 0.005;
	public static final double ELBOW_TOLERANCE = 0.01;

	
	// encoder failsafes
	private static final double MIN_WRIST_ANGLE = -0.35;
	private static final double MAX_WRIST_ANGLE = .4;
	private static final double MIN_ELBOW_ANGLE = -.15;
	private static final double MAX_ELBOW_ANGLE = 0.25;
	private static final double LOW_ENC_RATE = 0.005;
	private static final double MAX_PWM_THRESHOLD_NO_CURRENT = .1;
	public static final double MAX_UP_CURRENT_WRIST = 13;
	public static final double MAX_DOWN_CURRENT_WRIST = 13;
	public static final double MAX_UP_CURRENT_ELBOW = 12;
	public static final double MAX_DOWN_CURRENT_ELBOW = 8.5;
	
	public static final double QUASI_LOW_ELBOW_RATE = .01;
	
//	// current sensor failsafes
//	private static final double TALON_CURRENT_TOLERANCE = 10; //Should be 5
//	private static final double VICTOR_CURRENT_TOLERANCE = 1;
	

	public WarlordsPIDController elbowAngPID = new WarlordsPIDController();
	private WarlordsPIDController elbowAngVelPID = new WarlordsPIDController();
	public WarlordsPIDController wristAngPID = new WarlordsPIDController();
	private WarlordsPIDController wristAngVelPID = new WarlordsPIDController();

	private TransferNode elbowAngTN = new TransferNode(0);
	public TransferNode wristAngTN = new TransferNode(0);

	private PIDSourceWrapper kPElbowAngSource = new PIDSourceWrapper();
	private PIDSourceWrapper wristAngSource = new PIDSourceWrapper();
	private PIDSourceWrapper wristAngVelSource = new PIDSourceWrapper();
	private PIDSourceWrapper elbowMinCurrentSource = new PIDSourceWrapper();
	private PIDSourceWrapper elbowMaxCurrentSource = new PIDSourceWrapper();
	public PIDSourceWrapper wristMinCurrentSource = new PIDSourceWrapper();
	public PIDSourceWrapper wristMaxCurrentSource = new PIDSourceWrapper();
	public PIDSourceWrapper elbowMinAngVelSource = new PIDSourceWrapper();
	public PIDSourceWrapper elbowMaxAngVelSource = new PIDSourceWrapper();
	
	public double getVMaxElbow() {
		double thetaCritical = Math.acos((CRITICAL_DISTANCE - L2*Math.cos(getWristAngle() * 2 * Math.PI)) / L1) / Math.PI / 2;
		double vMax = ConstantsIO.kP_ElbowAng * (Math.abs(getElbowAngle()) - thetaCritical);
		vMax = Math.max(vMax, 0);
		return vMax;
	}
	

	public Arm() {

		new Timer().schedule(new CheckCurrentSensorTask(), 0, 20);
		// Elbow
		
		elbowMinAngVelSource.setPidSource(() -> {
			if (getElbowAngle() < 0) {
				return -0.2;
			}
			return -Math.min(getVMaxElbow(), 0.2);
		});
		
		elbowMaxAngVelSource.setPidSource(() -> {
			if (getElbowAngle() > 0) {
				return 0.3;
			}
			return Math.min(getVMaxElbow(), 0.3);
		});

		elbowAngPID.setSources(RobotMap.elbowEncoderWrapperDistance);
		elbowAngPID.setOutputs(elbowAngTN);
		elbowAngPID.setOutputSources(elbowMaxAngVelSource, elbowMinAngVelSource);
		
		
		elbowAngVelPID.setSources(RobotMap.elbowEncoderWrapperRate);
		elbowAngVelPID.setSetpointSource(elbowAngTN);
		elbowAngVelPID.setOutputs(RobotMap.elbowCurrentWrapper);
		elbowAngVelPID.setOutputSources(elbowMaxCurrentSource, elbowMinCurrentSource);

		elbowMinCurrentSource.setPidSource(() -> {
			if (Math.abs(RobotMap.elbowEncoderWrapperRate.pidGet()) < LOW_ENC_RATE) {
				return -2;
			}
			if (RobotMap.wristEncoderWrapperDistance.pidGet() > MAX_WRIST_ANGLE) { // if wrist at hard stop don't move arm down
				return -2;
			}
			return RobotMap.elbowEncoderWrapperDistance.pidGet() < MIN_ELBOW_ANGLE ? -2 : -MAX_DOWN_CURRENT_ELBOW;
		});
		elbowMaxCurrentSource.setPidSource(() -> {
			if (Math.abs(RobotMap.elbowEncoderWrapperRate.pidGet()) < LOW_ENC_RATE) {
				return 3;
			}
			return RobotMap.elbowEncoderWrapperDistance.pidGet() > MAX_ELBOW_ANGLE ? 2 : MAX_UP_CURRENT_ELBOW;
		});
		wristMinCurrentSource.setPidSource(() -> {
			if (Math.abs(RobotMap.wristEncoderWrapperRate.pidGet()) < LOW_ENC_RATE && Math.abs(RobotMap.elbowEncoderWrapperRate.pidGet()) < QUASI_LOW_ELBOW_RATE) {
				return -4;
			}
			return RobotMap.wristEncoderWrapperDistance.pidGet() < MIN_WRIST_ANGLE ? -2 : -MAX_DOWN_CURRENT_WRIST;
		});
		wristMaxCurrentSource.setPidSource(() -> {
			if (Math.abs(RobotMap.wristEncoderWrapperRate.pidGet()) < LOW_ENC_RATE && Math.abs(RobotMap.elbowEncoderWrapperRate.pidGet()) < QUASI_LOW_ELBOW_RATE) {
				return 4;
			}
			return RobotMap.wristEncoderWrapperDistance.pidGet() > MAX_WRIST_ANGLE ? 2 : MAX_UP_CURRENT_WRIST;
		});


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
		wristAngPID.setOutputRange(-.4, .4);

		wristAngVelPID.setSources(wristAngVelSource);
		wristAngVelPID.setSetpointSource(wristAngTN);
		wristAngVelPID.setOutputs(RobotMap.wristCurrentWrapper);
		wristAngVelPID.setOutputSources(wristMaxCurrentSource, wristMinCurrentSource);

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
		double theta2 = RobotMap.arm.getWristAngle();
    	if (RobotMap.arm.getElbowAngle() > 0) {
			RobotMap.arm.setThetaHigh(theta2);
		} else {
			RobotMap.arm.setThetaLow(theta2);
		}
		RobotMap.arm.setElbowPos(RobotMap.elbowEncoderWrapperDistance.pidGet());
    	elbowAngTN.setOutput(0);
    	wristAngTN.setOutput(0);
    	RobotMap.elbowCurrentWrapper.set(0);
    	RobotMap.wristCurrentWrapper.set(0);

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
		}
	}

	public void setWristManual(double pwm) {
		wristAngPID.disable();
		wristAngVelPID.disable();
		RobotMap.wristCurrentWrapper.set(pwm * ALPHA_MAX_WRIST);
	}

	public void setElbowManual(double pwm) {
		elbowAngPID.disable();
		elbowAngVelPID.disable();
		RobotMap.elbowCurrentWrapper.set(pwm * ALPHA_MAX_ELBOW);
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
			RobotMap.elbowCurrentWrapper.set(0);
		}
		if (wristPIDisEnabled()) {
			wristAngPID.disable();
			wristAngVelPID.disable();
			wristAngTN.setOutput(0);
			RobotMap.wristCurrentWrapper.set(0);
		}

		RobotMap.elbowCurrentWrapper.set(current);
	}

	public void setWristCurrent(double current) {
		if (elbowPIDisEnabled()) {
			elbowAngPID.disable();
			elbowAngVelPID.disable();
			elbowAngTN.setOutput(0);
			RobotMap.elbowCurrentWrapper.set(0);
		}
		if (wristPIDisEnabled()) {
			wristAngPID.disable();
			wristAngVelPID.disable();
			wristAngTN.setOutput(0);
			RobotMap.wristCurrentWrapper.set(0);
		}

		// wristSetter.disable();
		RobotMap.wristCurrentWrapper.set(current);
	}

	public void setElbowVelocity(double angVel) {
		if (!elbowPIDisEnabled()) {
			elbowAngVelPID.enable();
			elbowAngPID.disable();
		}

		elbowAngTN.setOutput(angVel);
	}

	public void setWristVelocity(double angVel) {
		if (!wristAngVelPID.isEnabled()) {
			wristAngVelPID.enable();
			wristAngPID.disable();
		}

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
		elbowAngPID.setPID(ConstantsIO.kP_ElbowAng, 0, 0);
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

	public class CheckCurrentSensorTask extends TimerTask {

		@Override
		public void run() {
			if(!isElbowCurrentSensorWorking()) {
				if (numBadCurrents > 5) {
					System.out.println("Elbow problem");
					Robot.forceDisable();
				}
				numBadCurrents++;
			} else if (!isWristCurrentSensorWorking()) {
				if (numBadCurrents > 5) {
					System.out.println("Wrist problem");
					Robot.forceDisable();
				}
				numBadCurrents++;
			} else {
				numBadCurrents = 0;
			}
		}
		
	}
}
