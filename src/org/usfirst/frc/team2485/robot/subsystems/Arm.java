package org.usfirst.frc.team2485.robot.subsystems;

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
	
	public static final double CRITICAL_DISTANCE = toMeters(34); //temp //distance from mast to 16 inches past frame perimeter 
	public static final double ALPHA_MAX_WRIST = 4;
	public static final double ALPHA_MAX_ELBOW = 4;
	public static final int ELBOW_OFFSET = -4053;
	public static final int WRIST_OFFSET = -852;
	
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
		INTAKE (-.18, 0),
		SWITCH (-.18, 0.15),
		SCALE (0.25, 0.1),
		SCALE_BACKWARDS (0, 0);
		
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
	
	
	/* arm = 1
	 * hand = 2
	 * L = length
	 * d = center of mass
	 * m = mass
	 * j = moment of inertia
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
	public static final double MAX_CURRENT_WRIST = 10;
	public static final double MAX_CURRENT_ELBOW = 10;

	public static final double J1 = toMetricInertia(I1) + (m1 * d1 * d1);
	public static final double J2 = toMetricInertia(I2) + (m2 * d2 * d2);
	
	public static final double torqueStall = .71;
	public static final double gearRatioWrist = 975; //temporary
	public static final double gearRatioElbow = 945;
	
	public static final double CRITICAL_ANGLE = Math.acos((CRITICAL_DISTANCE - L1) / L2) / 2 / Math.PI; //temp

	
	private WarlordsPIDController elbowAngPID = new WarlordsPIDController();
	private WarlordsPIDController elbowAngVelPID = new WarlordsPIDController();
	private WarlordsPIDController wristAngPID = new WarlordsPIDController();
	private WarlordsPIDController wristAngVelPID = new WarlordsPIDController();
	
	private TransferNode elbowAngTN = new TransferNode(0);
	private TransferNode elbowAngVelTN= new TransferNode(0);
	private TransferNode wristAngTN = new TransferNode(0);
	private TransferNode wristAngVelTN = new TransferNode(0);

	private PIDSourceWrapper wristAngSource = new PIDSourceWrapper();
	private PIDSourceWrapper wristAngVelSource = new PIDSourceWrapper();
	private PIDSourceWrapper elbowCurrentSource = new PIDSourceWrapper();
	private PIDSourceWrapper wristCurrentSource = new PIDSourceWrapper();
	
	private MotorSetter elbowSetter = new MotorSetter();
	private MotorSetter wristSetter = new MotorSetter();
	
	public Arm() {

//Elbow		
	
		elbowAngPID.setSources(RobotMap.elbowEncoderWrapperDistance);
		elbowAngPID.setOutputs(elbowAngTN);
		elbowAngPID.setOutputRange(-.1, .1);
		
		elbowAngVelPID.setSources(RobotMap.elbowEncoderWrapperRate);
		elbowAngVelPID.setSetpointSource(elbowAngTN);
		elbowAngVelPID.setOutputs(elbowAngVelTN);
		elbowAngVelPID.setOutputRange(-MAX_CURRENT_ELBOW, MAX_CURRENT_ELBOW);
		
		elbowCurrentSource.setPidSource(()-> {
			double a1 = elbowAngVelTN.pidGet();
//			double a2 = wristAngVelTN.pidGet();
//
//			double theta1 = getElbowAngle();
//			double theta2 = getWristAngle();
//			
//			double w1 = 2*Math.PI*RobotMap.elbowEncoderWrapperRate.pidGet();
//			double w2 = 2*Math.PI*RobotMap.wristEncoderWrapperRate.pidGet() + w1;
//			
//			double inertia = (J1 * a1) + 
////					(getJ2() * a2) + 
//					(L1*L1 * a1 * getM2()) + 
//					(L1 * a2 * getD2() * getM2() * Math.cos(theta1 - theta2));
//			double load = 
//					(L1 * g * getM2() * Math.cos(theta1)) +
//					(d1 * g * m1 * Math.cos(theta1)) +
//					(getD2() * g * getM2() * Math.cos(theta2));
////			double linking = (L1 * getD2() * getM2() * w2*w2 * Math.sin(theta1 - theta2));
//			double linking = 0;
//			double torque = inertia + load + linking;
//			double current = torque * (ConstantsIO.currentStallElbow / (2 * gearRatioElbow * torqueStall));
//			
//			return Math.min(Math.max(current, -MAX_CURRENT_ELBOW), MAX_CURRENT_ELBOW);
			return a1 + ConstantsIO.levitateElbowCurrent*Math.cos(RobotMap.elbowEncoderWrapperDistance.pidGet() * 2 * Math.PI) 
			+ ConstantsIO.levitateWristCurrent*Math.cos(wristAngSource.pidGet()*Math.PI*2)/5;
		});
		
		elbowSetter.setSetpointSource(elbowCurrentSource);
		elbowSetter.setOutputs(RobotMap.elbowCurrentWrapper);
		
//Wrist
		
		wristAngSource.setPidSource(() -> {
			return RobotMap.elbowEncoderWrapperDistance.pidGet() + RobotMap.wristEncoderWrapperDistance.pidGet();
		});
		
		wristAngVelSource.setPidSource(() -> {
			return RobotMap.elbowEncoderWrapperRate.pidGet() + RobotMap.wristEncoderWrapperRate.pidGet();
		});
		
		wristAngPID.setSources(wristAngSource);
		wristAngPID.setOutputs(wristAngTN);
		wristAngPID.setOutputRange(-.2, .2);
	
		
		wristAngVelPID.setSources(wristAngVelSource);
		wristAngVelPID.setSetpointSource(wristAngTN);
		wristAngVelPID.setOutputs(wristAngVelTN);
		wristAngVelPID.setOutputRange(-MAX_CURRENT_WRIST, MAX_CURRENT_WRIST);
		
		wristCurrentSource.setPidSource(()-> {
			
			double a2 = wristAngVelTN.pidGet();
			
//			double theta2 = 2*Math.PI*RobotMap.wristEncoderWrapperDistance.pidGet();
//			
//			double inertia = (getJ2() * a2);
//			double load = (getD2() * g * getM2() * Math.cos(theta2));
//			double torque = inertia + load;
//			double current = torque * (ConstantsIO.currentStallWrist / (gearRatioWrist * torqueStall));
//			return Math.min(Math.max(current, -MAX_CURRENT_WRIST), MAX_CURRENT_WRIST);
			return a2 + ConstantsIO.levitateWristCurrent*Math.cos(wristAngSource.pidGet()*Math.PI*2);
		});
		
		wristSetter.setSetpointSource(wristCurrentSource);
		wristSetter.setOutputs(RobotMap.wristCurrentWrapper);
		
	}
	
    public void initDefaultCommand() {
        setDefaultCommand(new WristWithControllers());
    }
    
    public double getM2() {
    	return RobotMap.intake.hasCube() ? m2 + mBox : m2;
    }
    
    public double getD2() {
    	double d2Box = (d2*m2 + LBox*mBox) / (m2 + mBox);
    	return RobotMap.intake.hasCube() ? d2Box : d2;
    }
    
    public double getJ2() {
    	return RobotMap.intake.hasCube() ? J2 + mBox*LBox*LBox : J2;
    }
   
    public void setWristPos(double pos) {
    	wristAngPID.enable();
    	wristAngVelPID.enable();
    	wristAngPID.setSetpoint(pos);
    	wristSetter.enable();
    	System.out.println(pos);
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
    	double margin = (CRITICAL_DISTANCE - (L1 * Math.cos(getElbowAngle()*2*Math.PI))) / L2;
    	return Math.abs(margin) < 1 ? 
    			Math.acos(margin) / 2 / Math.PI :
    				0;
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
    	RobotMap.elbowTalon.setSelectedSensorPosition(currPos, 0, 0);
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
    	RobotMap.wristTalon.setSelectedSensorPosition(currPos, 0, 0);
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
    
    //Testing
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
    
//    	wristSetter.disable();
    	RobotMap.wristCurrentWrapper.set(current);
    }
    
    public void setElbowVelocity(double angVel) {
    	if(!elbowPIDisEnabled()) {
    		elbowAngVelPID.enable();
    		elbowAngPID.disable();
    	}
    	
    	elbowSetter.enable();
    	elbowAngTN.setOutput(angVel);
    }
    
    public void setWristVelocity(double angVel) {
    	if(!wristAngVelPID.isEnabled()) {
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
    	return  wristAngVelPID.isEnabled(); // wristAngPID.isEnabled() &&
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
    	RobotMap.elbowTalon.configForwardSoftLimitEnable(true, 0);
    	RobotMap.elbowTalon.configReverseSoftLimitThreshold(ConstantsIO.kSoftLimitReverse_Elbow, 0);
    	RobotMap.elbowTalon.configReverseSoftLimitEnable(true, 0);

    	
    }
    
    public static double toMeters(double in) {
    	return in*0.0254;
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

