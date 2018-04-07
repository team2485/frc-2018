package org.usfirst.frc.team2485.util;

import java.io.FileWriter;



// @author larry mei and aditya gupta

import java.io.IOException;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commandGroups.ScaleAuto;
import org.usfirst.frc.team2485.robot.commands.ArmSetSetpoint;
import org.usfirst.frc.team2485.robot.subsystems.Arm.ArmSetpoint;
import org.usfirst.frc.team2485.util.Event.Type;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

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
Raw joystick values

 */

// Start with timeStamp, type, commandName, msg at first
public class AutoLogger {
	
	private static ArrayList<Event> savedEvents;
	
    private static final String COMMA_DELIMITER = ",";
    private static final String NEW_LINE_SEPARATOR = "\n";
    
	private static FileWriter fileWriter = null;

	public static void addEvent(Type type, String commandName, String msg) { // (Timeout/Completed) in typeMsg
		Event e = new Event(Timer.getFPGATimestamp(), type, commandName, msg); // replace null with getMsg(), make methods static in Event.java
		savedEvents.add(e);
	}
	
	
	
	
	
	public static ArrayList<Double> addData(double... data) {
		
		
		ArrayList<Double> datas = new ArrayList<Double>();
		for(int i=0; i<data.length-1; i++) {
			datas.add(data[i]);
		}
		
		return datas;
		
	}
	
	
	
	public static void write() {
		// Write values onto .csv file
		try {
			fileWriter = new FileWriter("Log " + System.currentTimeMillis() + ".csv");
			
			// Write the CSV file header
			fileWriter.append("Timestamp, Type, Command Name, Message");

			// Add a new line separator after the header
			fileWriter.append(NEW_LINE_SEPARATOR);
			
			//if it finishes normally "End Normally" if it timeouts put "timeout" if isFinished condition is true put "isFinished" else nothing

			// for loop to go through it then appending it to the .csv file
			for(Event e : savedEvents) {
				fileWriter.append(String.valueOf(e.getTimeStamp()));
				fileWriter.append(COMMA_DELIMITER);
				fileWriter.append(String.valueOf(e.getType()));
				fileWriter.append(COMMA_DELIMITER);
				fileWriter.append(e.getCommandName());
				fileWriter.append(COMMA_DELIMITER);
				fileWriter.append(e.getMsg());
				fileWriter.append(NEW_LINE_SEPARATOR);
			}

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
