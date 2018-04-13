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
		armUpWhenClose.addSequential(new WaitUntilClose(125));
		armUpWhenClose.addSequential(new ArmSetSetpoint(ArmSetpoint.SCALE_HIGH_BACK));

		if (startLeft == scaleLeft) {
			drive.addSequential(new ResetDriveTrain());
			AutoPath path = scaleLeft ? pathLeftStraight : pathRightStraight;
			drive.addSequential(new DriveTo(path, 300, true, 10000, true, true));
			isStraight = true;
		} else {
			AutoPath path = scaleLeft ? pathLeftCross : pathRightCross;
			drive.addSequential(new ResetDriveTrain());
			DriveTo crossPath = new DriveTo(path, 300, true, 100000, true, true);
			drive.addSequential(crossPath);
		}

		drive.addSequential(new ResetDriveTrain());
		CommandGroup ejecting = new CommandGroup();
		ejecting.addSequential(new WaitUntilClose(25));
		ejecting.addSequential(new Eject(true, true, true));
		firstCube.addParallel(drive);
		firstCube.addParallel(armUpWhenClose);
		firstCube.addParallel(ejecting);
		addSequential(firstCube);
		//addSequential(new Eject(true, true, true));
		addSequential(new ArmSetSetpoint(ArmSetpoint.INTAKE)); // intaking path: change setpoint to intake
		addSequential(new WaitForArm());
		addSequential(new DriveTo(scaleLeft ? intakePathLeft : intakePathRight, 300, false, 7000, false, true));
		addSequential(new ResetDriveTrain());
//		addSequential(new WaitForArm());
		addSequential(new SetIntakeManual(0.75));
		DriveTo driveStraight = new DriveTo(scaleLeft ? driveStraightLeft : driveStraightRight, 100, false, 6000, false, false);

		driveStraight.setFinishedCondition(() -> {
			return RobotMap.intake.isIntaken();
		});
		addSequential(driveStraight);
//		intaking.addSequential(new WaitUntilCubeIntaken(4000));
		addSequential(new ResetDriveTrain());
		addSequential(new StopIntaking());
//		addSequential(new TimedCommand(1));

//		if (switchLeft == scaleLeft) {
//			DriveStraight finalDrive = new DriveStraight(20, 30, 2000);
//			finalDrive.setTolerance(10);
//			addSequential(finalDrive);
//			addSequential(new Eject(true, true));
//		} else {
			addSequential(new ArmSetSetpoint(ArmSetpoint.SCALE_HIGH_BACK));
			addSequential(new DriveTo(scaleLeft ? driveStraightLeft : driveStraightRight, 100, true, 3000, false, false));
			addSequential(new ResetDriveTrain());
			addSequential(new DriveTo(scaleLeft ? intakePathLeft : intakePathRight, 300, true, 2500, false, false));
			addSequential(new Eject(false, true, true));
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
		
		driveStraightLeft = getDriveStraight(true);
		
		driveStraightRight = getDriveStraight(false);

	}

	public static AutoPath getCross(boolean left) {
		int sign = left ? -1 : 1;

		Pair[] controlPoints = { new Pair(-193 * sign, -270), new Pair(-214 * sign, -218), new Pair(0.0, -218), //193, -270,  
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
		Pair[] controlPoints = { new Pair(sign * 39.0, -245.0), new Pair(0, -115), new Pair(0.0, 0.0) };
		double[] dists = { 90.0, };
		return AutoPath.getAutoPathForClothoidSpline(controlPoints, dists);

	}

	public static AutoPath getIntakePath(boolean left) {
		int sign = left ? -1 : 1;
		AutoPath intakePath = new AutoPath(AutoPath.getPointsForBezier(2000, new Pair(sign * 6.0, 26.0),
				new Pair(sign * -1.0, 53.0), new Pair(sign * -5.9, 29.6), new Pair(sign * 2.1, 47.7)));
		return intakePath;

	}
	
	public static AutoPath getDriveStraight(boolean left) {
		AutoPath path = new AutoPath(AutoPath.getPointsForBezier(1000, new Pair(0, 0), new Pair(20 * Math.sin(left ? -.39 : .39), 20 * Math.cos(left ? -.39 : .39))));
		return path;
	}
	

}
