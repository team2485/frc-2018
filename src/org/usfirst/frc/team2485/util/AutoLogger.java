package org.usfirst.frc.team2485.util;

import java.io.FileWriter;

import java.io.IOException;

import java.util.ArrayList;

import java.util.List;

import org.usfirst.frc.team2485.robot.commandGroups.ScaleAuto;
import org.usfirst.frc.team2485.robot.commands.ArmSetSetpoint;
import org.usfirst.frc.team2485.robot.subsystems.Arm.ArmSetpoint;

import edu.wpi.first.wpilibj.Timer;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;

/*Create logging code for matches so that we have essential information to help us debug autos and teleop malfunctions during a match. Ensure that this does not take up too much CPU.

Things to record during auto:
Time of beginning and end of each command
X, Y coordinate
Encoder distance & rate
Gyro angle and rate
Arm and Wrist angle and rate

Things to record during auto:
Time of beginning and end of each command
Raw joystick values
X, Y coordinate
Arm and Wrist angle and rate */

// Start with timeStamp, type, commandName, msg at first
public class AutoLogger extends Event {

	public AutoLogger(double timeStamp, Type type, String commandName, String msg) {
		super(timeStamp, type, commandName, msg);
	}

	FileWriter fileWriter = null;

	public static void addEvent() {
		setTimeStamp(Timer.getFPGATimestamp());
		Event e = new Event(getTimeStamp(), getType(), getCommandName(), null); // replace null with getMsg(), make methods static in Event.java
	}

	public void read() {
		BufferedReader fileReader = null;
		try {
			ArrayList<AutoLogger> LoggerList = new ArrayList<AutoLogger>();
			String line = "";
			fileReader = new BufferedReader(new FileReader("AutoLogger.csv"));
			
			// Organize Data by individualizing them into columns
			
			// Print new organized data into list
//			for(AutoLogger LoggerList : )
			} catch (Exception e) {
			System.out.println("Error while reading CSV file");
			e.printStackTrace();
		} finally {
			try {
				fileReader.close();
			} catch (IOException e) {
				System.out.println("Error while closing fileReader");
				e.printStackTrace();
			}
		}
	}
	
	public void write() {
		// Write values onto .csv file
		try {
			fileWriter = new FileWriter("fileName");
			// Write the CSV file header
			fileWriter.append("AutoLogger");

			// Add a new line separator after the header
			fileWriter.append("\n");

			// Initialize class variables
			// addEvent(Timer.getFPGATimestamp(), );

		} catch (Exception e) {
			System.out.println("Error creating csvFileWriter");
			e.printStackTrace();
		} finally {
			try {
				fileWriter.flush();
				fileWriter.close();
			} catch (IOException e) {
				System.out.println("Error while closing csvFileWriter");
				e.printStackTrace();
			}
		}
	}
}
