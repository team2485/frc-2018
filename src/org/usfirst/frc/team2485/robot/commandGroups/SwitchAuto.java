package org.usfirst.frc.team2485.robot.commandGroups;

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
	public SwitchAuto(boolean left) {
		System.out.println("Switch Auto");
		addSequential(new ArmSetSetpoint(ArmSetpoint.SWITCH));
		int sign = left ? -1 : 1;
		AutoPath path = new AutoPath(AutoPath.getPointsForBezier(10000, new Pair(0.0, 0.0),
				new Pair(0, 44.0),
				new Pair(sign * 47.5, 50.0),
				new Pair(sign * 47.5, 99.5)));
		addSequential(new DriveTo(path, 90, false, 100000));
		addSequential(new ResetDriveTrain());
//		addSequential(new Eject(true));
		
	}
}
