package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.ArmSetSetpoint;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.ResetDriveTrain;
import org.usfirst.frc.team2485.robot.commands.WaitUntilClose;
import org.usfirst.frc.team2485.robot.subsystems.Arm.ArmSetpoint;
import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.AutoPath.Pair;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ScaleAuto extends CommandGroup {
	public ScaleAuto(boolean left) {
		CommandGroup drive = new CommandGroup();
		CommandGroup everythingElse = new CommandGroup();
		CommandGroup everythingBeforeEject = new CommandGroup();
		everythingElse.addSequential(new ArmSetSetpoint(ArmSetpoint.SWITCH));
		everythingElse.addSequential(new WaitUntilClose(120));
		everythingElse.addSequential(new ArmSetSetpoint(ArmSetpoint.SCALE_HIGH_BACK));
		
		int sign = left ? -1 : 1;
		Pair[] controlPoints = {new Pair(sign*65, -280), new Pair(0, -147), new Pair(0, 0)};
		double[] dists = {120};
		
		
		
		
		AutoPath path = AutoPath.getAutoPathForClothoidSpline(controlPoints, dists);
		drive.addSequential(new DriveTo(path, 75, true, 10000));
		drive.addSequential(new ResetDriveTrain());
		everythingBeforeEject.addParallel(drive);
		everythingBeforeEject.addParallel(everythingElse);
		addSequential(everythingBeforeEject);
		addSequential(new Eject(false, true));
		addSequential(new ArmSetSetpoint(ArmSetpoint.SWITCH));

	}
}
