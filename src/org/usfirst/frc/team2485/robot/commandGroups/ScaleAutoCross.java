package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.ArmSetSetpoint;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.ResetDriveTrain;
import org.usfirst.frc.team2485.robot.commands.WaitUntilClose;
import org.usfirst.frc.team2485.robot.subsystems.Arm.ArmSetpoint;
import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.AutoPath.Pair;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ScaleAutoCross extends CommandGroup {
	public ScaleAutoCross(boolean left) {
		CommandGroup drive = new CommandGroup();
		CommandGroup everythingElse = new CommandGroup();
		CommandGroup everythingBeforeEject = new CommandGroup();
		everythingElse.addSequential(new ArmSetSetpoint(ArmSetpoint.SWITCH));
		everythingElse.addSequential(new WaitUntilClose(120));
//		everythingElse.addSequential(new ArmSetSetpoint(ArmSetpoint.SCALE_HIGH_BACK));
		
		int sign = left ? -1 : 1;
		Pair[] controlPoints = {
//				new Pair(204.5, -289),
				new Pair(228.0, -212),
				new Pair(0.0, -212),
				new Pair(0.0, 0.0),
				};
				double[] dists = {100};
				AutoPath path = AutoPath.getAutoPathForClothoidSpline(controlPoints, dists);
		
		
		drive.addSequential(new DriveTo(path, 30, true, 190000));
		drive.addSequential(new ResetDriveTrain());
		everythingBeforeEject.addParallel(drive);
		everythingBeforeEject.addParallel(everythingElse);
		addSequential(everythingBeforeEject);
//		addSequential(new Eject(false, true));
//		addSequential(new ArmSetSetpoint(ArmSetpoint.SWITCH));

	}
}
