package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
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
	
	/* arm = 1
	 * hand = 2
	 * L = length
	 * d = center of mass
	 * m = mass
	 * j = moment of inertia
	 */
	
	public static final double J1 = toMetricInertia(289);
	public static final double J2 = toMetricInertia(565);
	public static final double L1 = toMeters(30);
	public static final double L2 = toMeters(28);
	public static final double d1 = toMeters(15);
	public static final double d2 = toMeters(19);
	public static final double m1 = toKilograms(3);
	public static final double m2 = toKilograms(10);
	public static final double g = 9.80665;
	public static final double PI = 3.141592653589;
	
	public static final double currentStall = 1; //temp
	public static final double torqueStall = 1;
	public static final double gearRatio = 1;
	
	private WarlordsPIDController elbowAngPID = new WarlordsPIDController();
	private WarlordsPIDController elbowAngVelPID = new WarlordsPIDController();
	private WarlordsPIDController wristAngPID = new WarlordsPIDController();
	private WarlordsPIDController wristAngVelPID = new WarlordsPIDController();
	
	private TransferNode elbowAngTN = new TransferNode(0);
	private TransferNode elbowAngVelTN= new TransferNode(0);
	private TransferNode wristAngTN = new TransferNode(0);
	private TransferNode wristAngVelTN = new TransferNode(0);

	private PIDSourceWrapper elbowCurrentSource;
	private PIDSourceWrapper wristCurrentSource;
	
	private MotorSetter elbowSetter;
	private MotorSetter wristSetter;
	
	public Arm() {
		updateConstants();

//Elbow		
	
		elbowAngPID.setSources(RobotMap.elbowEncoderWrapperDistance);
		elbowAngPID.setOutputs(elbowAngTN);
		
		elbowAngVelPID.setSources(RobotMap.elbowEncoderWrapperDistance);
		elbowAngVelPID.setSetpointSource(elbowAngTN);
		elbowAngVelPID.setOutputs(elbowAngVelTN);
		
		elbowCurrentSource.setPidSource(()-> {
			double a1 = elbowAngVelTN.pidGet();
			double a2 = wristAngVelTN.pidGet();
			double theta1 = 2*PI*RobotMap.elbowEncoderWrapperDistance.pidGet();
			double theta2 = 2*PI*RobotMap.wristEncoderWrapperDistance.pidGet();
			double w2 = 2*PI*RobotMap.wristEncoderWrapperRate.pidGet();
			
			double inertia = (J1 * a1) + 
					(J2 * a2) + 
					(L1*L1 * a1 * m2) + 
					(L1 * a2 * d2 * m2 * Math.cos(theta1 - theta2));
			double load = 
					(L1 * g * m2 * Math.cos(theta1)) +
					(d1 * g * m1 * Math.cos(theta1)) +
					(d2 * g * m2 * Math.cos(theta2));
			double linking = (L1 * d2 * m2 * w2*w2 * Math.sin(theta1 - theta2));
			double torque = inertia + load + linking;
			
			return torque * (currentStall / (2 * gearRatio * torqueStall));
		});
		
		elbowSetter.setSetpointSource(elbowCurrentSource);
		elbowSetter.setOutputs(RobotMap.elbowWrapper);
		
//Wrist
		
		wristAngPID.setSources(RobotMap.wristEncoderWrapperDistance);
		wristAngPID.setOutputs(wristAngTN);
		
		wristAngVelPID.setSources(RobotMap.wristEncoderWrapperDistance);
		wristAngVelPID.setSetpointSource(wristAngTN);
		wristAngVelPID.setOutputs(wristAngVelTN);
		
		wristCurrentSource.setPidSource(()-> {
			double a2 = wristAngVelTN.pidGet();
			double theta2 = 2*PI*RobotMap.wristEncoderWrapperDistance.pidGet();
			
			double inertia = (J2 * a2);
			double load = (d2 * g * m2 * Math.cos(theta2));
			double torque = inertia + load;
			
			return torque * (currentStall / (gearRatio * torqueStall));
		});
		
		wristSetter.setSetpointSource(wristCurrentSource);
		wristSetter.setOutputs(RobotMap.wristWrapper);
		
	}
	
    public void initDefaultCommand() {
        
    }
    
    
    public void updateConstants() {
    	elbowAngPID.setPID(ConstantsIO.kP_ElbowAng, 0, 0);
    	elbowAngVelPID.setPID(ConstantsIO.kP_ElbowAngVel, ConstantsIO.kI_ElbowAngVel, 0);
    	wristAngPID.setPID(ConstantsIO.kP_WristAng, 0, 0);
    	wristAngVelPID.setPID(ConstantsIO.kP_WristAng, ConstantsIO.kI_WristAngVel, 0);
    	
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
    
}

