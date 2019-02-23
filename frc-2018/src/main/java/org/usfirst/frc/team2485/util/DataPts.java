package org.usfirst.frc.team2485.util;

public class DataPts {
	private double timeStamp;
	private double[] dataPts = new double[] {};
	
	
	public DataPts(double timeStamp, double[] dataPts){
		this.dataPts = dataPts;
		this.timeStamp = timeStamp;
	}
	
	
	public double getTimeStamp() {	
		return timeStamp;
	}
	
	public double[] getDataPts() {
		return dataPts;
	}

}
