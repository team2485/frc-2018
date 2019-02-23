package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class G13Testing extends Command{

    public G13Testing(){
        super();

        requires(RobotMap.driveTrain);
        
        System.out.println("hi");
        setInterruptible(true);
    }

    protected void initialize() {

    }



    @Override
    protected boolean isFinished() {
        return false;
    }

}