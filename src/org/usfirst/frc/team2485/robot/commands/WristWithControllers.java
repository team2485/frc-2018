package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.Arm;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GearTooth;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class WristWithControllers extends Command {
	private static final double ELBOW_TOLERANCE = 0.0025;
	public static boolean isManual = true;
	public static boolean manualSetpoint = false;
	private double minWristPos;
	private int counter;
	
    public WristWithControllers() {
    	requires(RobotMap.arm);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	minWristPos = RobotMap.arm.getMinWristPos();
    	counter = 0;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	counter++;
    	double axis = -ThresholdHandler.deadbandAndScale(OI.operator.getRawAxis(OI.XBOX_LYJOYSTICK_PORT), OI.XBOX_AXIS_DEADBAND, 0, 1); // joystick command
    	if (counter % 4 == 0) {
    		minWristPos = RobotMap.arm.getMinWristPos();
    	}
    	double theta2 = RobotMap.wristEncoderWrapperDistance.pidGet();
    	if (DriverStation.getInstance().isAutonomous()) {
    		isManual = false;
    	} else if (axis != 0 && (axis > 0 || theta2 > minWristPos)) {
    		isManual = true;
    		RobotMap.arm.setWristManual(axis);
    	} else if (isManual) {
    		RobotMap.arm.setThetaWrist(theta2);
    		manualSetpoint = true;
    		isManual = false;
    	}
    	
    	
    	double[] thetasCritical = RobotMap.arm.getThetasCritical();
    	double theta1 = RobotMap.arm.getThetaElbow();
    	

    	if (thetasCritical[1] == 0 || RobotMap.arm.isClimb) {
    		RobotMap.arm.setElbowSetpoint(theta1);
    	} else if (RobotMap.arm.getThetaElbow() < thetasCritical[0]) {
    		RobotMap.arm.setElbowSetpoint(Math.min(thetasCritical[0] - thetasCritical[1], theta1));
    	} else {
    		RobotMap.arm.setElbowSetpoint(Math.max(thetasCritical[0] + thetasCritical[1], theta1));

    	}

   
    	if (!isManual) {
    		if (RobotMap.arm.isClimb) {
    			double angle = Math.max(Math.min(RobotMap.arm.getThetaWrist(), .23 - RobotMap.arm.getElbowAngle()), -RobotMap.arm.getElbowAngle());
    			RobotMap.arm.setWristPos(manualSetpoint ? RobotMap.arm.getThetaWrist() : angle);
    		} else if ((RobotMap.arm.getElbowAngle() - Arm.MID_ELBOW_ANGLE) * (RobotMap.arm.getElbowSetpoint() - Arm.MID_ELBOW_ANGLE) < 0) { // crossing from low to high
    			RobotMap.arm.setWristPos(Math.max(Arm.MIN_WRIST_ANGLE_CROSS, RobotMap.arm.getThetaWrist()));
    		} else {
    			RobotMap.arm.setWristPos(Math.max(RobotMap.arm.getThetaWrist(), minWristPos));
    		}
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
