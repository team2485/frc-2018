package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.ArmSetSetpoint;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.ResetDriveTrain;
import org.usfirst.frc.team2485.robot.commands.SetIntakeManual;
import org.usfirst.frc.team2485.robot.subsystems.Arm.ArmSetpoint;
import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.AutoPath.Pair;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;

public class SwitchAuto extends CommandGroup {
	private static AutoPath leftPath, rightPath;
	public SwitchAuto(boolean left) {
		addSequential(new ArmSetSetpoint(ArmSetpoint.SWITCH));
		AutoPath path = left ? leftPath : rightPath;
//		RobotMap.pathTracker.start(path);
		addSequential(new DriveTo(path, 60, false, 9000, false));
		addSequential(new ResetDriveTrain());
		addSequential(new Eject(true, false));
		
	}
	public static void init() {
		leftPath = getAutoPath(true);
		rightPath = getAutoPath(false);
	}
	
	private static AutoPath getAutoPath(boolean left) { 
		int sign = left ? -1 : 1;
		return new AutoPath(AutoPath.getPointsForBezier(10000, new Pair(0.0, 0.0),
				new Pair(0, 44.0),
				new Pair(sign * 55.5, 30.0),
				new Pair(sign * 55.5, 106)));
	}
}
