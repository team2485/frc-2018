package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.ArmSetSetpoint;
import org.usfirst.frc.team2485.robot.commands.DriveStraight;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.ResetDriveTrain;
import org.usfirst.frc.team2485.robot.commands.RotateTo;
import org.usfirst.frc.team2485.robot.commands.SetIntakeManual;
import org.usfirst.frc.team2485.robot.commands.StopIntaking;
import org.usfirst.frc.team2485.robot.commands.Wait;
import org.usfirst.frc.team2485.robot.commands.WaitUntilArmUp;
import org.usfirst.frc.team2485.robot.commands.WaitUntilClose;
import org.usfirst.frc.team2485.robot.commands.WaitUntilCubeIntaken;
import org.usfirst.frc.team2485.robot.subsystems.Arm.ArmSetpoint;
import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.AutoPath.Pair;
import org.usfirst.frc.team2485.util.FinishedCondition;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ScaleAuto extends CommandGroup {
	public static AutoPath pathLeftCross, pathRightCross, pathLeftStraight, pathRightStraight, intakePathLeft,
			intakePathRight;
	boolean isStraight = false;

	public ScaleAuto(boolean startLeft, boolean switchLeft, boolean scaleLeft) {
		CommandGroup drive = new CommandGroup();
		CommandGroup everythingElse = new CommandGroup();
		CommandGroup everythingBeforeEject = new CommandGroup();
		everythingElse.addSequential(new ArmSetSetpoint(ArmSetpoint.SWITCH));
		everythingElse.addSequential(new WaitUntilClose(140));
		everythingElse.addSequential(new ArmSetSetpoint(ArmSetpoint.SCALE_HIGH_BACK));

		if (startLeft == scaleLeft) {
			drive.addSequential(new ResetDriveTrain());
			AutoPath path = scaleLeft ? pathLeftStraight : pathRightStraight;
			drive.addSequential(new DriveTo(path, 85, true, 10000, false));
			isStraight = true;
		} else {
			AutoPath path = scaleLeft ? pathLeftCross : pathRightCross;
			DriveTo crossPath = new DriveTo(path, 75, true, 100000, false);
			drive.addSequential(new ResetDriveTrain());
			drive.addSequential(crossPath);
		}

		drive.addSequential(new ResetDriveTrain());
		everythingBeforeEject.addParallel(drive);
		everythingBeforeEject.addParallel(everythingElse);
		addSequential(everythingBeforeEject);
		addSequential(new Eject(false, true));
		addSequential(new ArmSetSetpoint(ArmSetpoint.INTAKE)); // intaking path: change setpoint to intake
		addSequential(new WaitUntilArmUp());

		CommandGroup getCube = new CommandGroup();
		CommandGroup driveToIntake = new CommandGroup();
		CommandGroup intaking = new CommandGroup();
		driveToIntake.addSequential(new DriveTo(scaleLeft ? intakePathLeft : intakePathRight, 50, false, 8000, false));
		driveToIntake.addSequential(new ResetDriveTrain());
		intaking.addSequential(new SetIntakeManual(.75));
		intaking.addSequential(new WaitUntilCubeIntaken());
		intaking.addSequential(new StopIntaking());
		intaking.addSequential(new ArmSetSetpoint(ArmSetpoint.SWITCH));
		getCube.addParallel(driveToIntake);
		getCube.addParallel(intaking);
		addSequential(getCube);
		if (switchLeft == scaleLeft) {
			DriveStraight finalDrive = new DriveStraight(20, 30, 2000);
			finalDrive.setTolerance(10);
			addSequential(finalDrive);
			addSequential(new Eject(true, true));
		} 

	}

	public static void init() {

		pathLeftCross = getCross(true);

		pathRightCross = getCross(false);

		pathLeftStraight = getStraight(true);

		pathRightStraight = getStraight(false);

		intakePathLeft = getIntakePath(true);

		intakePathRight = getIntakePath(false);

	}

	public static AutoPath getCross(boolean left) {
		int sign = left ? -1 : 1;

		Pair[] controlPoints = { new Pair(-197.0 * sign, -280), new Pair(-215.0 * sign, -220), new Pair(0.0, -220),
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
		Pair[] controlPoints = { new Pair(sign * 39.0, -280.0), new Pair(0, -150), new Pair(0.0, 0.0) };
		double[] dists = { 90.0, };
		return AutoPath.getAutoPathForClothoidSpline(controlPoints, dists);

	}

	public static AutoPath getIntakePath(boolean left) {
		int sign = left ? -1 : 1;
		AutoPath intakePath = new AutoPath(AutoPath.getPointsForBezier(2000, new Pair(6.0, 0.0),
				new Pair(sign * -1.0, 27.0), new Pair(sign * -3.0, 44.0), new Pair(sign * 6.5, 71.0)));
		return intakePath;

	}

}
