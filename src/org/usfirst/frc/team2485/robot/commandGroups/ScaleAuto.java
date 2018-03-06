package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.ArmSetSetpoint;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.ResetDriveTrain;
import org.usfirst.frc.team2485.robot.commands.WaitUntilClose;
import org.usfirst.frc.team2485.robot.subsystems.Arm.ArmSetpoint;
import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.AutoPath.Pair;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ScaleAuto extends CommandGroup {
	public static AutoPath pathLeftCross, pathRightCross, pathLeftStraight, pathRightStraight;

	public ScaleAuto(boolean startLeft, boolean scaleLeft) {
		CommandGroup drive = new CommandGroup();
		CommandGroup everythingElse = new CommandGroup();
		CommandGroup everythingBeforeEject = new CommandGroup();
		everythingElse.addSequential(new ArmSetSetpoint(ArmSetpoint.SWITCH));
		everythingElse.addSequential(new WaitUntilClose(120));
		everythingElse.addSequential(new ArmSetSetpoint(ArmSetpoint.SCALE_HIGH_BACK));
		
		if(startLeft == scaleLeft && scaleLeft) {
			drive.addSequential(new DriveTo(pathLeftStraight, 75, true, 100000));
		} else if (startLeft == scaleLeft && !scaleLeft) {
			drive.addSequential(new DriveTo(pathRightStraight, 75, true, 100000));
		} else if(startLeft != scaleLeft && scaleLeft) {
			drive.addSequential(new DriveTo(pathLeftCross, 30, true, 100000));
		} else {
			drive.addSequential(new DriveTo(pathRightCross, 30, true, 100000));
		}
		
		drive.addSequential(new ResetDriveTrain());
		everythingBeforeEject.addParallel(drive);
		everythingBeforeEject.addParallel(everythingElse);
		addSequential(everythingBeforeEject);
		addSequential(new Eject(false, true));
		addSequential(new ArmSetSetpoint(ArmSetpoint.SWITCH));

	}

	public static void init() {

		pathLeftCross = getCross(true);

		pathRightCross = getCross(false);

		pathLeftStraight = getStraight(true);
		
		pathRightStraight = getStraight(false);
	}
	
	public static AutoPath getCross(boolean left) {
		int sign = left ? -1 : 1;
		Pair[] controlPoints = {new Pair(sign*-209, -299), new Pair(sign*-260.0, -212), new Pair(0.0, -212), new Pair(0.0, 0.0),};
		double[] dists = { 50, 100 };
		
		
		return AutoPath.getAutoPathForClothoidSpline(controlPoints, dists);
	}
	
	public static AutoPath getStraight(boolean left) {
		int sign = left ? -1 : 1;
		Pair[] controlPoints = { new Pair(sign*65, -280), new Pair(0, -147), new Pair(0, 0) };
		double[] dists = { 120 };
		return AutoPath.getAutoPathForClothoidSpline(controlPoints, dists);

	}

}
