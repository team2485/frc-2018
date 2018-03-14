package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.ArmSetSetpoint;
import org.usfirst.frc.team2485.robot.commands.DriveStraight;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.ResetDriveTrain;
import org.usfirst.frc.team2485.robot.commands.Wait;
import org.usfirst.frc.team2485.robot.subsystems.Arm.ArmSetpoint;
import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.AutoPath.Pair;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class SwitchAuto extends CommandGroup {
	private static AutoPath leftPath, rightPath;
	public SwitchAuto(boolean left) {
		addSequential(new ArmSetSetpoint(ArmSetpoint.SWITCH));
		AutoPath path = left ? leftPath : rightPath;
//		RobotMap.pathTracker.start(path);
		DriveTo drive = new DriveTo(path, 60, false, 8000, false);
		drive.setDistTolerance(5);
		addSequential(drive);
		addSequential(new ResetDriveTrain());
		addSequential(new Eject(true, false));
		addSequential(new DriveStraight(-30, 50, 5000));
		
	}
	public static void init() {
		leftPath = getAutoPath(true);
		rightPath = getAutoPath(false);
	}
	
	private static AutoPath getAutoPath(boolean left) { 
		int sign = left ? -1 : 1;
		return new AutoPath(AutoPath.getPointsForBezier(2000, new Pair(0.0, 0.0),
				new Pair(0, 44.0),
				new Pair(sign * 53.5 - 6, 30.0),
				new Pair(sign * 53.5 - 6, 104)), 
				AutoPath.getPointsForBezier(2000, 
						new Pair(sign * 53.5 - 6, 104), 
						new Pair(sign * 53.5 - 6, 120)));
	}
}
