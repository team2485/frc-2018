package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.ArmSetSetpoint;
import org.usfirst.frc.team2485.robot.commands.CancelCommand;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.ResetDriveTrain;
import org.usfirst.frc.team2485.robot.commands.SetIntakeManual;
import org.usfirst.frc.team2485.robot.commands.StopIntaking;
import org.usfirst.frc.team2485.robot.commands.Wait;
import org.usfirst.frc.team2485.robot.commands.WaitForArm;
import org.usfirst.frc.team2485.robot.commands.WaitUntilClose;
import org.usfirst.frc.team2485.robot.commands.ZeroEncoders;
import org.usfirst.frc.team2485.robot.subsystems.Arm.ArmSetpoint;
import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.AutoPath.Pair;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;

public class ScaleAuto extends CommandGroup {
	public static AutoPath pathLeftCross, pathRightCross, pathLeftStraight, pathRightStraight, intakePathLeft,
			intakePathRight, driveStraightLeft, driveStraightRight;
	boolean isStraight = false;

	public ScaleAuto(boolean startLeft, boolean switchLeft, boolean scaleLeft) {
		CommandGroup drive = new CommandGroup();
		CommandGroup armUpWhenClose = new CommandGroup();
		CommandGroup firstCube = new CommandGroup();
		armUpWhenClose.addSequential(new ArmSetSetpoint(ArmSetpoint.SWITCH));
		armUpWhenClose.addSequential(new WaitUntilClose(100));
		armUpWhenClose.addSequential(new ArmSetSetpoint(ArmSetpoint.SCALE_HIGH_BACK));

		if (startLeft == scaleLeft) {
			drive.addSequential(new ResetDriveTrain());
			AutoPath path = scaleLeft ? pathLeftStraight : pathRightStraight;
			drive.addSequential(new DriveTo(path, 300, true, 10000, true, true));
			isStraight = true;
		} else {
			AutoPath path = scaleLeft ? pathLeftCross : pathRightCross;
			drive.addSequential(new ResetDriveTrain());
			DriveTo crossPath = new DriveTo(path, 300, true, 7500, true, true);
			crossPath.setDistTolerance(60);
			drive.addSequential(crossPath);
		}

		drive.addSequential(new ResetDriveTrain());
		CommandGroup ejecting = new CommandGroup();
		ejecting.addSequential(new WaitUntilClose(100));
		ejecting.addSequential(new Eject(true, true, true));
		firstCube.addParallel(drive);
		firstCube.addParallel(armUpWhenClose);
		firstCube.addParallel(ejecting);
		addSequential(firstCube);
		addSequential(new ArmSetSetpoint(ArmSetpoint.INTAKE)); // intaking path: change setpoint to intake
		addSequential(new TimedCommand(.7));
		DriveTo intakePath = new DriveTo(scaleLeft ? intakePathLeft : intakePathRight, 300, false, 2000, false, true);
		intakePath.setAngleTolerance(.175);
		addSequential(intakePath);
		addSequential(new ResetDriveTrain());
		addSequential(new SetIntakeManual(0.75));
		DriveTo driveStraight = new DriveTo(scaleLeft ? driveStraightLeft : driveStraightRight, 100, false, 7500, false,
				false);

		driveStraight.setFinishedCondition(() -> {
			return RobotMap.intake.isIntaken();
		});
		addSequential(driveStraight);
		// intaking.addSequential(new WaitUntilCubeIntaken(4000));
		addSequential(new ResetDriveTrain());
		addSequential(new StopIntaking());
		// addSequential(new TimedCommand(1));

		// if (switchLeft == scaleLeft) {
		// DriveStraight finalDrive = new DriveStraight(20, 30, 2000);
		// finalDrive.setTolerance(10);
		// addSequential(finalDrive);
		// addSequential(new Eject(true, true));
		// } else {
		addSequential(new ArmSetSetpoint(ArmSetpoint.SCALE_HIGH_BACK));
		addSequential(new DriveTo(scaleLeft ? driveStraightLeft : driveStraightRight, 100, true, 1000, false, false));
		addSequential(new ResetDriveTrain());
		DriveTo secondCube = new DriveTo(scaleLeft ? intakePathLeft : intakePathRight, 300, true, 1250, false, false);
		secondCube.setDistTolerance(40);
		secondCube.setAngleTolerance(.175);
		addSequential(secondCube);
		addSequential(new Eject(true, true, true));
		addSequential(new ArmSetSetpoint(ArmSetpoint.SWITCH));
		// }

	}

	public static void init() {

		pathLeftCross = getCross(true);

		pathRightCross = getCross(false);

		pathLeftStraight = getStraight(true);

		pathRightStraight = getStraight(false);

		intakePathLeft = getIntakePath(true);

		intakePathRight = getIntakePath(false);

		driveStraightLeft = getDriveStraight(true);

		driveStraightRight = getDriveStraight(false);

	}

	public static AutoPath getCross(boolean left) {
		int sign = left ? -1 : 1;

		Pair[] controlPoints = { new Pair(-193 * sign, -255), new Pair(-214 * sign, -200), new Pair(0.0, -200), // 193,
																												// -270,
				new Pair(0.0, 0.0), };
		double[] dists = { 50.0, 75.0, };
		return AutoPath.getAutoPathForClothoidSpline(controlPoints, dists);
		// Pair[] controlPoints = {new Pair(sign*-209, -299), new Pair(sign*-260.0,
		// -212), new Pair(0.0, -212), new Pair(0.0, 0.0),};
		// double[] dists = { 50, 100 };
		//
		//
		// return AutoPath.getAutoPathForClothoidSpline(controlPoints, dists);
	}

	public static AutoPath getStraight(boolean left) {
		int sign = left ? -1 : 1;
		 Pair[] controlPoints = { new Pair(sign * 50.4, -283.0), new Pair(0, -115), new Pair(0.0, 0.0) };
		
		 double[] dists = { 90.0, };
		return AutoPath.getAutoPathForClothoidSpline(controlPoints, dists);

	}

	public static AutoPath getIntakePath(boolean left) {
		int sign = left ? -1 : 1;
		 AutoPath intakePath = new AutoPath(AutoPath.getPointsForBezier(2000, new Pair(sign * -5.4, -12.0),
		 new Pair(sign * -6.88, -6.12), new Pair(sign * -6.16, 28.32), new Pair(sign *
		 -4.1, 32.7)));
		
		return intakePath;

	}

	public static AutoPath getDriveStraight(boolean left) {
		AutoPath path = new AutoPath(AutoPath.getPointsForBezier(1000, new Pair(0, 0),
				new Pair(0 /** Math.sin(left ? -.39 : .39)*/, 20 /*** Math.cos(left ? -.39 : .39)*/)));
		return path;
	}

}
