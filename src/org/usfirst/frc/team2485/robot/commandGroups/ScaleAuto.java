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
import org.usfirst.frc.team2485.robot.commands.WaitForArm;
import org.usfirst.frc.team2485.robot.commands.WaitUntilClose;
import org.usfirst.frc.team2485.robot.commands.WaitUntilCubeIntaken;
import org.usfirst.frc.team2485.robot.commands.ZeroEncoders;
import org.usfirst.frc.team2485.robot.subsystems.Arm.ArmSetpoint;
import org.usfirst.frc.team2485.robot.subsystems.Intake;
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
		everythingElse.addSequential(new WaitUntilClose(200));
		everythingElse.addSequential(new ArmSetSetpoint(ArmSetpoint.SCALE_HIGH_BACK));

		if (startLeft == scaleLeft) {
			drive.addSequential(new ResetDriveTrain());
			AutoPath path = scaleLeft ? pathLeftStraight : pathRightStraight;
			drive.addSequential(new ZeroEncoders());
			drive.addSequential(new DriveTo(path, 307, true, 10000, true));
			isStraight = true;
		} else {
			AutoPath path = scaleLeft ? pathLeftCross : pathRightCross;
			drive.addSequential(new ResetDriveTrain());
			drive.addSequential(new ZeroEncoders());
			DriveTo crossPath = new DriveTo(path, 307, true, 100000, true);
			drive.addSequential(crossPath);
		}

		drive.addSequential(new ResetDriveTrain());
		everythingBeforeEject.addParallel(drive);
		everythingBeforeEject.addParallel(everythingElse);
		addSequential(everythingBeforeEject);
		addSequential(new Eject(false, true));
		addSequential(new ArmSetSetpoint(ArmSetpoint.INTAKE)); // intaking path: change setpoint to intake
		addSequential(new ZeroEncoders());
		addSequential(new DriveTo(scaleLeft ? intakePathLeft : intakePathRight, 307, false, 7000, false));
		addSequential(new ResetDriveTrain());
		addSequential(new WaitForArm());
		addSequential(new SetIntakeManual(0.75));
		addSequential(new ZeroEncoders());
		DriveStraight driveStraight = new DriveStraight(16, scaleLeft ? -.39 : .39, 50, 6000);

		driveStraight.setFinishedCondition(() -> {
			return RobotMap.intake.isIntaken();
		});
		addSequential(driveStraight);
//		intaking.addSequential(new WaitUntilCubeIntaken(4000));
		addSequential(new ResetDriveTrain());
		addSequential(new StopIntaking());
//		if (switchLeft == scaleLeft) {
//			DriveStraight finalDrive = new DriveStraight(20, 30, 2000);
//			finalDrive.setTolerance(10);
//			addSequential(finalDrive);
//			addSequential(new Eject(true, true));
//		} else {
			addSequential(new ArmSetSetpoint(ArmSetpoint.SCALE_HIGH_BACK));
			addSequential(new DriveTo(scaleLeft ? intakePathLeft : intakePathRight, 307, true, 20000, false));
			addSequential(new ResetDriveTrain());
			addSequential(new Eject(false, true));
			addSequential(new ArmSetSetpoint(ArmSetpoint.SWITCH));
//		}

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

		Pair[] controlPoints = { new Pair(-197 * sign, -280), new Pair(-218 * sign, -210), new Pair(0.0, -210),
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
		Pair[] controlPoints = { new Pair(sign * 39.0, -275.0), new Pair(0, -145), new Pair(0.0, 0.0) };
		double[] dists = { 90.0, };
		return AutoPath.getAutoPathForClothoidSpline(controlPoints, dists);

	}

	public static AutoPath getIntakePath(boolean left) {
		int sign = left ? -1 : 1;
		AutoPath intakePath = new AutoPath(AutoPath.getPointsForBezier(2000, new Pair(sign * 6.0, 0.0),
				new Pair(sign * -1.0, 27.0), new Pair(sign * -3.8, 35.0), new Pair(sign * 4.7, 53.1)));
		return intakePath;

	}
	

}
