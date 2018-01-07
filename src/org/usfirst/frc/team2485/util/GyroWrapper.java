package org.usfirst.frc.team2485.util;

import org.usfirst.frc.team2485.robot.Robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;

public class GyroWrapper implements PIDSource {

	private AHRS ahrs;
	private AnalogGyro simulationGyro;

	public GyroWrapper() {
		if(Robot.isSimulation()){
			simulationGyro = new AnalogGyro(10);
			simulationGyro.reset();
		} else {
			ahrs = new AHRS(SPI.Port.kMXP);
			ahrs.reset();
		}
	}

	@Override
	public PIDSourceType getPIDSourceType() {

		return PIDSourceType.kDisplacement;
	}

	@Override
	public double pidGet() {
		// TODO Auto-generated method stub
		return getYaw();
	}

	@Override
	public void setPIDSourceType(PIDSourceType arg0) {


	}

	public double getYaw(){
		if(Robot.isSimulation()){
			return simulationGyro.getAngle();
		} else {
			return ahrs.getYaw();
		}
	}
	
	public void reset(){
		if(Robot.isSimulation()){
			simulationGyro.reset();
		} else {
			ahrs.reset();
		}
	
	}

}
