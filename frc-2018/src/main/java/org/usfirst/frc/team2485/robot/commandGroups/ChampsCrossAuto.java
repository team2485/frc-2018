package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.ArmSetSetpoint;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.ResetDriveTrain;
import org.usfirst.frc.team2485.robot.subsystems.Arm.ArmSetpoint;
import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.AutoPath.Pair;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ChampsCrossAuto extends CommandGroup {
	private static AutoPath scaleLeftPath;
	private static AutoPath scaleRightPath;

    public ChampsCrossAuto(boolean startLeft, boolean scaleLeft) {
    	addSequential(new ArmSetSetpoint(ArmSetpoint.SWITCH));
    	AutoPath path = scaleLeft ? scaleLeftPath : scaleRightPath;
		addSequential(new ResetDriveTrain());
		DriveTo crossPath = new DriveTo(path, 50, true, 7500, true, true);
		crossPath.setDistTolerance(25);
		addSequential(crossPath);
		addSequential(new ResetDriveTrain());
    }
    
    public static void init() {
    	scaleLeftPath = getCross(true);
    	scaleRightPath = getCross(false);
    }
    
    public static AutoPath getCross(boolean left) {
		int sign = left ? -1 : 1;

		Pair[] controlPoints = {new Pair(-100 * sign, -215), new Pair(0.0, -215), 																	
				new Pair(0.0, 0.0), };
		double[] dists = { 75.0};
		return AutoPath.getAutoPathForClothoidSpline(controlPoints, dists);
		// Pair[] controlPoints = {new Pair(sign*-209, -299), new Pair(sign*-260.0,
		// -212), new Pair(0.0, -212), new Pair(0.0, 0.0),};
		// double[] dists = { 50, 100 };
		//
		//
		// return AutoPath.getAutoPathForClothoidSpline(controlPoints, dists);
	}
}
