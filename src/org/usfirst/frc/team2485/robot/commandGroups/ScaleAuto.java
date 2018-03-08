package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.ArmSetSetpoint;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.ResetDriveTrain;
import org.usfirst.frc.team2485.robot.commands.RotateTo;
import org.usfirst.frc.team2485.robot.commands.SetIntakeManual;
import org.usfirst.frc.team2485.robot.commands.StopIntaking;
import org.usfirst.frc.team2485.robot.commands.Wait;
import org.usfirst.frc.team2485.robot.commands.WaitUntilArmUp;
import org.usfirst.frc.team2485.robot.commands.WaitUntilClose;
import org.usfirst.frc.team2485.robot.subsystems.Arm.ArmSetpoint;
import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.AutoPath.Pair;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ScaleAuto extends CommandGroup {
	public static AutoPath pathLeftCross, pathRightCross, pathLeftStraight, pathRightStraight, intakePathLeft, intakePathRight;
	boolean isStraight = false;

	public ScaleAuto(boolean startLeft, boolean scaleLeft) {
		CommandGroup drive = new CommandGroup();
		CommandGroup everythingElse = new CommandGroup();
		CommandGroup everythingBeforeEject = new CommandGroup();
		everythingElse.addSequential(new ArmSetSetpoint(ArmSetpoint.SWITCH));
		everythingElse.addSequential(new WaitUntilClose(120));
		everythingElse.addSequential(new ArmSetSetpoint(ArmSetpoint.SCALE_HIGH_BACK));
		
		if(startLeft == scaleLeft) {
			AutoPath path = scaleLeft ? pathLeftStraight : pathRightStraight;
			drive.addSequential(new DriveTo(path, 100, true, 100000, true));
			isStraight = true;
		} else {
			AutoPath path = scaleLeft ? pathLeftCross : pathRightCross;
			DriveTo crossPath = new DriveTo(path, 100, true, 100000, true);
			crossPath.setAngleTolerance(.2);
			drive.addSequential(crossPath);
			drive.addSequential(new ResetDriveTrain());
			drive.addSequential(new RotateTo(scaleLeft ? 0.866 : -0.866, 10000));
		} 
		
		drive.addSequential(new ResetDriveTrain());
		everythingBeforeEject.addParallel(drive);
		everythingBeforeEject.addParallel(everythingElse);
		addSequential(everythingBeforeEject);
		addSequential(new Eject(false, true));
		if (!isStraight) {
			addSequential(new ArmSetSetpoint(ArmSetpoint.SWITCH));
		} else {
			addSequential(new ArmSetSetpoint(ArmSetpoint.INTAKE));

			addSequential(new WaitUntilArmUp());

			CommandGroup getCube = new CommandGroup();
			CommandGroup driveToIntake = new CommandGroup();
			CommandGroup intaking = new CommandGroup();
			driveToIntake.addSequential(new DriveTo(scaleLeft ? intakePathLeft : intakePathRight, 20, false, 6000, false));
			driveToIntake.addSequential(new ResetDriveTrain());
			intaking.addSequential(new SetIntakeManual(.6));
			intaking.addSequential(new Wait(() -> {
				return RobotMap.intake.hasCube();
			}));
			intaking.addSequential(new StopIntaking());
			getCube.addParallel(driveToIntake);
			getCube.addParallel(intaking);
			addSequential(getCube);
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
		
		Pair[] controlPoints = {
				new Pair(-218 * sign, -302),
				new Pair(-190 * sign, -212),
				new Pair(0, -212),
				new Pair(0.0, 0.0),
		};
		double[] dists = {62, 100};
		return AutoPath.getAutoPathForClothoidSpline(controlPoints, dists);
//		Pair[] controlPoints = {new Pair(sign*-209, -299), new Pair(sign*-260.0, -212), new Pair(0.0, -212), new Pair(0.0, 0.0),};
//		double[] dists = { 50, 100 };
//		
//		
//		return AutoPath.getAutoPathForClothoidSpline(controlPoints, dists);
	}
	
	public static AutoPath getStraight(boolean left) {
		int sign = left ? -1 : 1;
		Pair[] controlPoints = { new Pair(sign*45, -280), new Pair(0, -147), new Pair(0, 0) };
		double[] dists = { 120 };
		return AutoPath.getAutoPathForClothoidSpline(controlPoints, dists);

	}
	
	public static AutoPath getIntakePath(boolean left) {
		int sign = left ? -1 : 1;
		AutoPath intakePath = new AutoPath(AutoPath.getPointsForBezier(2000, new Pair(sign*-17.5, -71.5), new Pair(sign*-28.5, -39.0), new Pair(sign * -27.2, -46.69), new Pair(sign * -5.2, 0.13)));
		return intakePath;
		
	}

}
