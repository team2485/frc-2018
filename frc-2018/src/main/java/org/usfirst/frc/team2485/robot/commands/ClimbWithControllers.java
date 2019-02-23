package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.Robot;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ClimbWithControllers extends Command {

    public ClimbWithControllers() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(RobotMap.climber);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }
    
    @Override
    protected void execute() {
    	// TODO Auto-generated method stub
    	super.execute();
    	double pwm = ThresholdHandler.deadbandAndScale(OI.operator.getRawAxis(OI.XBOX_LTRIGGER_PORT), OI.XBOX_TRIGGER_DEADBAND, 0, 1);
    	if (pwm == 0) {
    		pwm = ThresholdHandler.deadbandAndScale(OI.operatorBackup.getRawAxis(OI.XBOX_LTRIGGER_PORT), OI.XBOX_TRIGGER_DEADBAND, 0, 1);
    	}
    	if(!RobotMap.arm.isClimb) {
    		pwm = 0;
    	}
    	RobotMap.climber.setManual(pwm);
    }

    // Called once after isFinished returns true
    protected void end() {
    }

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

}
