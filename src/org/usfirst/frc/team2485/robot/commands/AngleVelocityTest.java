package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AngleVelocityTest extends Command {
	private double maxAngVel, minAngVel, rate;
	private long period;
	private int executeTime = 20;
	private double currAngVel = 0; //ms
	private int direction = 1;
	
	/**
	 * 
	 * @param maxAngVel
	 * @param minAngVel
	 * @param period in seconds
	 */
    public AngleVelocityTest(double maxAngVel, double minAngVel, long period) {
    	this.maxAngVel = maxAngVel;
    	this.minAngVel = minAngVel;
    	this.period = period;
    	rate = ((maxAngVel - minAngVel) / (period/2)) * executeTime;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (currAngVel >= maxAngVel || currAngVel <= minAngVel) {
    		direction *= -1;
    	}
    	currAngVel += direction * rate;
    	RobotMap.driveTrain.setAngVel(currAngVel);
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
