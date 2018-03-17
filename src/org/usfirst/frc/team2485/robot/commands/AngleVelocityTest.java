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
	private long lastTime = 0;
	
	/**
	 * 
	 * @param maxAngVel
	 * @param minAngVel
	 * @param period in milliseconds
	 */
    public AngleVelocityTest(double maxAngVel, double minAngVel, long period) {
    	requires(RobotMap.driveTrain);
    	this.maxAngVel = maxAngVel;
    	this.minAngVel = minAngVel;
    	this.period = period;
    	currAngVel = (minAngVel + maxAngVel) / 2;
    	rate = ((maxAngVel - minAngVel) / (period/2));
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	lastTime = System.currentTimeMillis();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double deltaTime = System.currentTimeMillis() - lastTime;
    	if (currAngVel >= maxAngVel) {
    		direction = -1;
    	} else if (currAngVel <= minAngVel) {
    		direction = 1;
    	}
    	currAngVel += direction * rate * deltaTime;
    	RobotMap.driveTrain.setVelocities(30, currAngVel);
    	lastTime = System.currentTimeMillis();

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
