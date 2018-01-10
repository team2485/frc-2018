package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.MotorSetter;
import org.usfirst.frc.team2485.util.PIDSourceWrapper;
import org.usfirst.frc.team2485.util.PigeonWrapperRateAndAngle;
import org.usfirst.frc.team2485.util.PigeonWrapperRateAndAngle.Units;
import org.usfirst.frc.team2485.util.TransferNode;
import org.usfirst.frc.team2485.util.WarlordsPIDController;

import com.ctre.phoenix.motorcontrol.ControlMode;

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


	public static final double STEERING_DEADBAND = 0.15;
	public static final double THROTTLE_DEADBAND = 0.05;
	private static final double MIN_CURRENT = 2;
	private static final double MAX_CURRENT = 20;
	
	private boolean isQuickTurn;
	
	private WarlordsPIDController distancePID = new WarlordsPIDController();
	private WarlordsPIDController anglePID = new WarlordsPIDController();
	private WarlordsPIDController velocityPID = new WarlordsPIDController();
	private WarlordsPIDController angVelocityPID = new WarlordsPIDController();
	
	private TransferNode distanceTN = new TransferNode(0);
	private TransferNode angleTN = new TransferNode(0);
	private TransferNode velocityTN = new TransferNode(0);
	private TransferNode angVelocityTN = new TransferNode(0);
	
	private TransferNode curvatureTN = new TransferNode(0);
	
	private PIDSourceWrapper distancePIDSource = new PIDSourceWrapper();
	private PIDSourceWrapper velocityPIDSource = new PIDSourceWrapper();
	
	private PIDSourceWrapper encoderDistancePIDSource = new PIDSourceWrapper();
	private PIDSourceWrapper encoderAvgVelocityPIDSource = new PIDSourceWrapper();
	
	private PIDSourceWrapper leftPIDSource = new PIDSourceWrapper();
	private PIDSourceWrapper rightPIDSource = new PIDSourceWrapper();
	
	private PIDSourceWrapper angVelTargSource = new PIDSourceWrapper();
	
	private MotorSetter leftMotorSetter = new MotorSetter();
	private MotorSetter rightMotorSetter = new MotorSetter();
	
	
	
    public DriveTrain() {
    	
//    	velocityPIDSource.setPidSource(() -> {
//    		return ;
//    	});
//
//    	deltaVelocityPIDSource.setPidSource(() -> {
//    		if ()
//    	});
    	
    	
    	//distance
    	distancePIDSource.setPidSource(() -> {
    		return Math.min(ConstantsIO.kP_Distance, 
    				(2*ConstantsIO.accelerationMax) / 
    				((RobotMap.driveLeftEncoderWrapperRate.pidGet() + RobotMap.driveRightEncoderWrapperRate.pidGet()) / 2)) ;
    	});
    	
    	encoderDistancePIDSource.setPidSource(() -> {
    		return (RobotMap.driveLeftEncoderWrapperDistance.pidGet() + RobotMap.driveRightEncoderWrapperDistance.pidGet()) / 2;
    	});
    	
    	distancePID.setOutputs(distanceTN);
    	distancePID.setSources(encoderDistancePIDSource);

    	encoderAvgVelocityPIDSource.setPidSource(() -> {
    		return (RobotMap.driveLeftEncoderWrapperRate.pidGet() + RobotMap.driveRightEncoderWrapperRate.pidGet()) / 2;
    	});
    	velocityPID.setOutputs(velocityTN);
    	velocityPID.setSources(encoderAvgVelocityPIDSource);
    	velocityPID.setSetpointSource(distanceTN);
    	
    	//angle
    	
    	anglePID.setOutputs(angleTN);
    	anglePID.setSources(RobotMap.pigeonDisplacementWrapper);
    	
    	angVelTargSource.setPidSource(() -> {
    		return angleTN.pidGet() + (curvatureTN.pidGet() * encoderAvgVelocityPIDSource.pidGet());
    	});
    	
    	angVelocityPID.setSetpointSource(angVelTargSource);
    	angVelocityPID.setOutputs(angVelocityTN);
    	angVelocityPID.setSources(RobotMap.pigeonRateWrapper);
    	
    	leftPIDSource.setPidSource(() -> {
    		return velocityTN.pidGet() + angVelocityTN.pidGet();
    	});
    	rightPIDSource.setPidSource(() -> {
    		return velocityTN.pidGet() - angVelocityTN.pidGet();
    	});
    	
    	leftMotorSetter.setSources(leftPIDSource);
    	leftMotorSetter.setOutputs(RobotMap.driveLeft);
    	rightMotorSetter.setSources(rightPIDSource);
    	rightMotorSetter.setOutputs(RobotMap.driveRight);
    	
    }
    
    public void initDefaultCommand() {
        
    }
    
    public void WARLordsDrive() {
    	
    }
    
    public void setDriveSpeed(DriveSpeed speed) {
		driveSpeed = speed.getSpeedFactor();
	}
    
    public void zeroEncoders() {
		
		RobotMap.driveLeftEncoderWrapperDistance.reset();
		RobotMap.driveRightEncoderWrapperDistance.reset();

	}
    
    public void reset() {
		
		RobotMap.driveLeft.set(0);
		RobotMap.driveRight.set(0);
		
	}
    
    public void setLeftRightCurrent(double l, double r) {
		
		RobotMap.driveLeftTalon1.set(ControlMode.Current, l);
		RobotMap.driveLeftTalon2.set(ControlMode.Current, l);
		RobotMap.driveLeftTalon3.set(ControlMode.Current, l);
		RobotMap.driveRightTalon1.set(ControlMode.Current, r);
		RobotMap.driveRightTalon2.set(ControlMode.Current, r);
		RobotMap.driveRightTalon3.set(ControlMode.Current, r);
	}
    
	public void setLeftRightVelocity(double l, double r) {
		
		RobotMap.driveLeftTalon1.set(ControlMode.Velocity, l);
		RobotMap.driveLeftTalon2.set(ControlMode.Velocity, l);
		RobotMap.driveLeftTalon3.set(ControlMode.Velocity, l);
		RobotMap.driveRightTalon1.set(ControlMode.Velocity, r);
		RobotMap.driveRightTalon2.set(ControlMode.Velocity, r);
		RobotMap.driveRightTalon3.set(ControlMode.Velocity, r);
	}
}
