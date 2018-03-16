package org.usfirst.frc.team2485.robot.subsystems;

import java.util.Timer;
import java.util.TimerTask;

import org.usfirst.frc.team2485.robot.Robot;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.WristWithControllers;
import org.usfirst.frc.team2485.util.ConstantsIO;
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
		INTAKE(-.192, -0.015), SWITCH(-.192, 0.15), SECOND_STORY(-.192, 0.04), SCALE_HIGH_BACK(0.16, 0.39), SCALE_MIDDLE_BACK(0.16, 0.46), SCALE_LOW_BACK(0.16, 0.5), SEVEN_FOOT_SCALE(.26, .124);

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
	private static final double MIN_ELBOW_ANGLE = -.1;
	private static final double MAX_ELBOW_ANGLE = 0.25;
	private static final double LOW_ENC_RATE = 0.005;
	private static final double MAX_PWM_THRESHOLD_NO_CURRENT = .1;
	public static final double MAX_UP_CURRENT_WRIST = 20;
	public static final double MAX_DOWN_CURRENT_WRIST = 20;
	public static final double MAX_UP_CURRENT_ELBOW = 25;
	public static final double MAX_DOWN_CURRENT_ELBOW = 20;
	
	public static final double VMAX_WRIST = .4;
	public static final double VMIN_WRIST = -.4;
	public static final double VMAX_ELBOW = .2;
	public static final double VMIN_ELBOW = -.15;
	
	public static final double QUASI_LOW_ELBOW_RATE = .01;
	
	

	public WarlordsPIDController elbowAngPID = new WarlordsPIDController();
	private WarlordsPIDController elbowAngVelMaxPID = new WarlordsPIDController();
	private WarlordsPIDController elbowAngVelMinPID = new WarlordsPIDController();
	public WarlordsPIDController wristAngPID = new WarlordsPIDController();
	private WarlordsPIDController wristAngVelMaxPID = new WarlordsPIDController();
	private WarlordsPIDController wristAngVelMinPID = new WarlordsPIDController();

	public TransferNode wristAngVelMaxTN = new TransferNode(0);
	public TransferNode wristAngVelMinTN = new TransferNode(0);
	public TransferNode elbowAngVelMaxTN = new TransferNode(0);
	public TransferNode elbowAngVelMinTN = new TransferNode(0);

	private PIDSourceWrapper wristAngSource = new PIDSourceWrapper();
	public PIDSourceWrapper wristMinCurrentSource = new PIDSourceWrapper();
	public PIDSourceWrapper wristMaxCurrentSource = new PIDSourceWrapper();
	public PIDSourceWrapper elbowMinAngVelSource = new PIDSourceWrapper();
	public PIDSourceWrapper elbowMaxAngVelSource = new PIDSourceWrapper();
	public PIDSourceWrapper wristMaxAngSource = new PIDSourceWrapper();
	public PIDSourceWrapper wristMinAngSource = new PIDSourceWrapper();
	public PIDSourceWrapper elbowMaxAngSource = new PIDSourceWrapper();
	public PIDSourceWrapper elbowMinAngSource = new PIDSourceWrapper();

	
	
	public double getVMaxElbow() {
		double thetaCritical = Math.acos((CRITICAL_DISTANCE - L2*Math.cos(getWristAngle() * 2 * Math.PI)) / L1) / Math.PI / 2;
		double vMax = ConstantsIO.kP_ElbowAng * (Math.abs(getElbowAngle()) - thetaCritical);
		vMax = Math.max(vMax, 0);
		return vMax;
	}
	
	public double getIMaxElbow() {
		double i = RobotMap.elbowTalon.getOutputCurrent();
		double pwm = RobotMap.elbowTalon.getMotorOutputPercent();
		double iMax = MAX_UP_CURRENT_ELBOW;
		return Math.min(iMax, Math.abs(i/pwm));
	}
	
	public double getIMinElbow() {
		double i = RobotMap.elbowTalon.getOutputCurrent();
		double pwm = RobotMap.elbowTalon.getMotorOutputPercent();
		double iMax = MAX_DOWN_CURRENT_ELBOW;
		return -Math.min(iMax, Math.abs(i/pwm));
	}
	
	public double getIMaxWrist() {
		double i = RobotMap.wristTalon.getOutputCurrent();
		double pwm = RobotMap.wristTalon.getMotorOutputPercent();
		double iMax = MAX_UP_CURRENT_WRIST;
		return Math.min(iMax, Math.abs(i/pwm));
	}
	
	public double getIMinWrist() {
		double i = RobotMap.wristTalon.getOutputCurrent();
		double pwm = RobotMap.wristTalon.getMotorOutputPercent();
		double iMax = MAX_DOWN_CURRENT_WRIST;
		return -Math.min(iMax, Math.abs(i/pwm));
	}
	

	public Arm() {

		new Timer().schedule(new CheckCurrentSensorTask(), 0, 20);
		// Elbow
		elbowMaxAngSource.setPidSource(() -> {
			return elbowAngVelMaxTN.pidGet();
		});
		
		elbowMinAngSource.setPidSource(() -> {
			return elbowAngVelMinTN.pidGet();
		});
		
		elbowAngVelMaxPID.setSources(RobotMap.elbowEncoderWrapperRate);
		elbowAngVelMaxPID.setOutputRange(0, getIMaxElbow());
		elbowAngVelMaxPID.setOutputs(elbowAngVelMaxTN);
		
		elbowAngVelMinPID.setSources(RobotMap.elbowEncoderWrapperRate);
		elbowAngVelMinPID.setOutputRange(getIMinElbow(), 0);
		elbowAngVelMinPID.setOutputs(elbowAngVelMinTN);
		
		elbowAngPID.setSources(RobotMap.elbowEncoderWrapperDistance);
		elbowAngPID.setOutputSources(elbowMaxAngSource, elbowMinAngSource);
		elbowAngPID.setOutputs(RobotMap.elbowCurrentWrapper);
		
		//Wrist
		wristMaxAngSource.setPidSource(() -> {
			return wristAngVelMaxTN.pidGet();
		});
		
		wristMinAngSource.setPidSource(() -> {
			return wristAngVelMinTN.pidGet();
		});
		
		wristAngVelMaxPID.setSources(RobotMap.wristEncoderWrapperRate);
		wristAngVelMaxPID.setOutputRange(0, getIMaxWrist());
		wristAngVelMaxPID.setOutputs(wristAngVelMaxTN);
		
		wristAngVelMinPID.setSources(RobotMap.wristEncoderWrapperRate);
		wristAngVelMinPID.setOutputRange(getIMinWrist(), 0);
		wristAngVelMinPID.setOutputs(elbowAngVelMinTN);
		
		wristAngPID.setSources(RobotMap.wristEncoderWrapperDistance);
		wristAngPID.setOutputSources(wristMaxAngSource, wristMinAngSource);
		wristAngPID.setOutputs(RobotMap.wristCurrentWrapper);

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
		wristAngVelMaxPID.setSetpoint(VMAX_WRIST);
		wristAngVelMinPID.setSetpoint(VMIN_WRIST);
		if (Math.abs(pos - wristAngSource.pidGet()) < WRIST_TOLERANCE) {
			setWristManual(0);
		} else {
			wristAngPID.enable();
			wristAngVelMaxPID.enable();
			wristAngVelMinPID.enable();
		}
	}

	public void setWristManual(double pwm) {
		wristAngPID.disable();
		wristAngVelMaxPID.disable();
		RobotMap.wristCurrentWrapper.set(pwm * ALPHA_MAX_WRIST);
	}

	public void setElbowManual(double pwm) {
		elbowAngPID.disable();
		elbowAngVelMaxPID.disable();
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
		elbowAngVelMaxPID.enable();
		elbowAngVelMinPID.enable();
		elbowAngVelMaxPID.setSetpoint(VMAX_ELBOW);
		elbowAngVelMinPID.setSetpoint(VMIN_ELBOW);
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
			elbowAngVelMaxPID.disable();
			RobotMap.elbowCurrentWrapper.set(0);
		}
		if (wristPIDisEnabled()) {
			wristAngPID.disable();
			wristAngVelMaxPID.disable();
			RobotMap.wristCurrentWrapper.set(0);
		}

		RobotMap.elbowCurrentWrapper.set(current);
	}

	public void setWristCurrent(double current) {
		if (elbowPIDisEnabled()) {
			elbowAngPID.disable();
			elbowAngVelMaxPID.disable();
			RobotMap.elbowCurrentWrapper.set(0);
		}
		if (wristPIDisEnabled()) {
			wristAngPID.disable();
			wristAngVelMaxPID.disable();
			RobotMap.wristCurrentWrapper.set(0);
		}

		// wristSetter.disable();
		RobotMap.wristCurrentWrapper.set(current);
	}


	public double getWristAngVelMaxError() {
		return wristAngVelMaxPID.getAvgError();
	}
	
	public double getWristAngVelMinError() {
		return wristAngVelMinPID.getAvgError();
	}

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

	public boolean wristPIDisEnabled() {
		return wristAngVelMaxPID.isEnabled(); // wristAngPID.isEnabled() &&
	}

	public boolean elbowPIDisEnabled() {
		return elbowAngPID.isEnabled() && elbowAngVelMaxPID.isEnabled();
	}

	public void updateConstants() {
		elbowAngPID.setPID(ConstantsIO.kP_ElbowAng, 0, 0);
		elbowAngVelMaxPID.setPID(ConstantsIO.kP_ElbowAngVelMax, ConstantsIO.kI_ElbowAngVelMax, 0, ConstantsIO.kF_ElbowAngVelMax);
		elbowAngVelMinPID.setPID(ConstantsIO.kP_ElbowAngVelMin, ConstantsIO.kI_ElbowAngVelMin, 0, ConstantsIO.kF_ElbowAngVelMin);
		wristAngPID.setPID(ConstantsIO.kP_WristAng, 0, 0);
		wristAngVelMaxPID.setPID(ConstantsIO.kP_WristAngVelMax, ConstantsIO.kI_WristAngVelMax, 0, ConstantsIO.kF_WristAngVelMax);
		wristAngVelMinPID.setPID(ConstantsIO.kP_WristAngVelMin, ConstantsIO.kI_WristAngVelMin, 0, ConstantsIO.kF_WristAngVelMin);
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
