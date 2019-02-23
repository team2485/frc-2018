package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.RobotMap;
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
	private static AutoPath leftStraightIntakePath, rightStraightIntakePath;
	private static AutoPath backup;
	public SwitchAuto(boolean switchLeft, boolean scaleLeft) {
		setRunWhenDisabled(true);
		setInterruptible(true);
		addSequential(new ArmSetSetpoint(ArmSetpoint.SWITCH));
		AutoPath path = switchLeft ? leftPath : rightPath;
//		RobotMap.pathTracker.start(path);
		DriveTo drive = new DriveTo(path, 120, false, 6000, false, true);
		drive.setDistTolerance(20);
		addSequential(drive);
		addSequential(new ResetDriveTrain());
		addSequential(new Eject(true, false, true));
		addSequential(new DriveTo(backup, 120, true, 2500, false, true));
		addSequential(new ResetDriveTrain());
		addSequential(new ArmSetSetpoint(ArmSetpoint.INTAKE));
		addSequential(new SetIntakeManual(.75));
		addSequential(new DriveTo(switchLeft ? intakeLeftPath : intakeRightPath, 50, false, 3500, true, true));
		addSequential(new ResetDriveTrain());
//		intaking.addSequential(new WaitUntilCubeIntaken(6000));
		DriveTo straightIntake = new DriveTo(switchLeft ? leftStraightIntakePath : rightStraightIntakePath, 100, false, 100000, false, false);
		straightIntake.setFinishedCondition(() -> {
			return RobotMap.intake.isIntaken();
		});
		addSequential(straightIntake);
		addSequential(new ResetDriveTrain());
		addSequential(new StopIntaking());
		addSequential(new ArmSetSetpoint(ArmSetpoint.SWITCH));
		addSequential(new DriveTo(switchLeft ? intakeLeftPath : intakeRightPath, 50, true, 10000, false, true));
		addSequential(new ResetDriveTrain());
		addSequential(new DriveTo(backup, 120, false, 10000, false, true));
		addSequential(new Eject(true, false, true));
//		addSequential(new DriveTo(switchLeft ? intakeLeftPath : intakeRightPath, 80, false, 3500, false, false));
//		DriveStraight straightPath = new DriveStraight(80, 120, 2500);
//		straightPath.setTolerance(10);
//		addSequential(straightPath);
//		addSequential(new Eject(true, true, true));
		
//		AutoPath pathToScale;
//		CommandGroup drivingToScale = new CommandGroup();
//		if (scaleLeft == switchLeft) {
//			pathToScale = switchLeft ? leftToScaleLeft : rightToScaleRight;
//		} else {
//			pathToScale = switchLeft ? leftToScaleRight : rightToScaleLeft;
//		}
//		drivingToScale.addSequential(new DriveTo(pathToScale, 80, true, 8000, false, true));
//		drivingToScale.addSequential(new ResetDriveTrain());
//		addSequential(drivingToScale);
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
		leftStraightIntakePath = getStraightIntakePath(true);
		rightStraightIntakePath = getStraightIntakePath(false);
		backup = getBackup();
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
				new Pair(sign * -37.0, -54.0), new Pair(sign * -45.5, -35.0)));
		
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
	
	public static AutoPath getBackup() {
		return new AutoPath(AutoPath.getPointsForBezier(2000, new Pair(0, 0), new Pair(0, 70)));
	}
	
	public static AutoPath getStraightIntakePath(boolean switchLeft) {
		return new AutoPath(AutoPath.getPointsForBezier(2000, new Pair(0, 0), new Pair(Math.sin(switchLeft ? 0.42 : -0.42) * 50, Math.cos(switchLeft ? 0.42 : -0.42) * 50)));
	}
	
	protected void interrupted() {
		// TODO Auto-generated method stub
		super.interrupted();
		this.reset();
	}
	
	@Override
	public synchronized void cancel() {
		// TODO Auto-generated method stub
		super.cancel();
		this.reset();
	}
	
	@Override
	protected void end() {
		// TODO Auto-generated method stub
		super.end();
		this.reset();
	}
	
	private void reset() {
		// TODO Auto-generated method stub
		RobotMap.driveTrain.reset();
		RobotMap.arm.reset();
		RobotMap.intake.setRollers(0);
	}
}
