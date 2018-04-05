package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.ArmSetSetpoint;
import org.usfirst.frc.team2485.robot.commands.DriveStraight;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.ResetDriveTrain;
import org.usfirst.frc.team2485.robot.commands.SetIntakeManual;
import org.usfirst.frc.team2485.robot.commands.StopIntaking;
import org.usfirst.frc.team2485.robot.commands.WaitForArm;
import org.usfirst.frc.team2485.robot.commands.WaitUntilClose;
import org.usfirst.frc.team2485.robot.commands.WaitUntilCubeIntaken;
import org.usfirst.frc.team2485.robot.subsystems.Arm.ArmSetpoint;
import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.AutoPath.Pair;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class SwitchAuto extends CommandGroup {
	private static AutoPath leftPath, rightPath;
	private static AutoPath intakeLeftPath, intakeRightPath;
	private static AutoPath leftToScaleLeft, rightToScaleLeft, leftToScaleRight, rightToScaleRight;
	public SwitchAuto(boolean switchLeft, boolean scaleLeft) {
		addSequential(new ArmSetSetpoint(ArmSetpoint.SWITCH));
		AutoPath path = switchLeft ? leftPath : rightPath;
//		RobotMap.pathTracker.start(path);
		DriveTo drive = new DriveTo(path, 70, false, 6000, false);
		drive.setDistTolerance(20);
		addSequential(drive);
		addSequential(new ResetDriveTrain());
		addSequential(new Eject(true, false));
		addSequential(new DriveStraight(-70, 0, 120, 2500));
		addSequential(new ResetDriveTrain());
		addSequential(new ArmSetSetpoint(ArmSetpoint.INTAKE));
		CommandGroup getCube = new CommandGroup();
		CommandGroup driveToIntake = new CommandGroup();
		CommandGroup intaking = new CommandGroup();
		driveToIntake.addSequential(new DriveTo(switchLeft ? intakeLeftPath : intakeRightPath, 80, false, 3500, false));
		driveToIntake.addSequential(new ResetDriveTrain());
		intaking.addSequential(new SetIntakeManual(.75));
		intaking.addSequential(new WaitUntilCubeIntaken(6000));
		intaking.addSequential(new StopIntaking());
		intaking.addSequential(new ArmSetSetpoint(ArmSetpoint.SWITCH));
		getCube.addParallel(driveToIntake);
		getCube.addParallel(intaking);
		addSequential(getCube);
//		addSequential(new DriveTo(left ? intakeLeftPath : intakeRightPath, 80, true, 3500, false));
//		DriveStraight straightPath = new DriveStraight(80, 120, 2500);
//		straightPath.setTolerance(10);
//		addSequential(straightPath);
//		addSequential(new Eject(true, true));
		AutoPath pathToScale;
		CommandGroup drivingToScale = new CommandGroup();
		if (scaleLeft == switchLeft) {
			pathToScale = switchLeft ? leftToScaleLeft : rightToScaleRight;
		} else {
			pathToScale = switchLeft ? leftToScaleRight : rightToScaleLeft;
		}
		drivingToScale.addSequential(new DriveTo(pathToScale, 80, true, 8000, false));
		drivingToScale.addSequential(new ResetDriveTrain());
		addSequential(drivingToScale);
	}

	public static void init() {
		leftPath = getAutoPath(true);
		rightPath = getAutoPath(false);
		intakeLeftPath = getIntakePath(true);
		intakeRightPath = getIntakePath(false);
		leftToScaleLeft = getToScalePathStraight(true);
		rightToScaleRight = getToScalePathStraight(false);
		rightToScaleLeft = getToScalePathCross(false);
		leftToScaleRight = getToScalePathCross(true);
	}

	private static AutoPath getAutoPath(boolean left) {
		int sign = left ? -1 : 1;
		return new AutoPath(
				AutoPath.getPointsForBezier(2000, new Pair(0.0, 0.0), new Pair(0, 44.0),
						new Pair(sign * 53.5 - 6, 30.0), new Pair(sign * 53.5 - 6, 94)),
				AutoPath.getPointsForBezier(2000, new Pair(sign * 53.5 - 6, 99), new Pair(sign * 53.5 - 6, 115)));
	}

	private static AutoPath getIntakePath(boolean left) {
		int sign = left ? -1 : 1;
		return new AutoPath(AutoPath.getPointsForBezier(2000, new Pair(0.0, -70), new Pair(0.0, -40.0),
				new Pair(sign * -33.0, -54.0), new Pair(sign * -41.5, -35.0)));
	}
	
	public static AutoPath getToScalePathStraight(boolean left) {
		int sign = left ? -1 : 1;
		Pair[] controlPoints = {new Pair(0.0, 0.0), new Pair(0.0, -217.0), new Pair(-sign * 77.75, -217.0), new Pair(-sign * 99.5, -167.0)};
		double[] dists = {36, 18};
		AutoPath path = AutoPath.getAutoPathForClothoidSpline(controlPoints, dists);
		return path;
	}
	
	public static AutoPath getToScalePathCross(boolean switchLeft) {
		int sign = switchLeft ? -1 : 1;
		Pair[] controlPoints = {new Pair(0.0, 0.0), new Pair(0.0, -217.0), new Pair(sign * 142.75, -217.0), new Pair(sign * 121.5, -167.0)};
		double[] dists = {68.5, 73.0};
		AutoPath path = AutoPath.getAutoPathForClothoidSpline(controlPoints, dists);
		
		return path;
	}
}
