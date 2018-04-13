package org.usfirst.frc.team2485.util;

import java.io.FileWriter;



// @author larry mei and aditya gupta

import java.io.IOException;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
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

	private static ArrayList<Event> savedEvents = new ArrayList<Event>();
	private static ArrayList<DataPts> savedDataPts = new ArrayList<DataPts>();

	private static final String COMMA_DELIMITER = ",";
	private static final String NEW_LINE_SEPARATOR = "\n";

	private static FileWriter fileWriterEvents = null;
	private static FileWriter fileWriterDataPts = null;

	public static void addEvent(Type type, String commandName, String msg) { // (Timeout/Completed) in typeMsg
		Event e = new Event(Timer.getFPGATimestamp(), type, commandName, msg); // replace null with getMsg(), make methods static in Event.java
		savedEvents.add(e);
	}

	public static void addDataPt(double... datas) {
		DataPts p = new DataPts(Timer.getFPGATimestamp(), datas);
		savedDataPts.add(p);
	}

	public static void write() {

		// Write values onto .csv file
		try {
			String date = new Date().toString();
			if (savedEvents.size() > 0) {
				fileWriterEvents = new FileWriter("/home/lvuser/Events/Log " + date + ".csv");

				// Write the CSV file header
				fileWriterEvents.append("Timestamp, Type, Command Name, Message");

				// Add a new line separator after the header
				fileWriterEvents.append(NEW_LINE_SEPARATOR);

				//if it finishes normally "End Normally" if it timeouts put "timeout" if isFinished condition is true put "isFinished" else nothing

				// for loop to go through it then appending it to the .csv file
				for(Event e : savedEvents) {
					fileWriterEvents.append(String.valueOf(e.getTimeStamp()));
					fileWriterEvents.append(COMMA_DELIMITER);
					fileWriterEvents.append(String.valueOf(e.getType()));
					fileWriterEvents.append(COMMA_DELIMITER);
					fileWriterEvents.append(e.getCommandName());
					fileWriterEvents.append(COMMA_DELIMITER);
					fileWriterEvents.append(e.getMsg());
					fileWriterEvents.append(NEW_LINE_SEPARATOR);
				}
				fileWriterEvents.flush();
				fileWriterEvents.close();
			}

			if (savedDataPts.size() > 0) {
				fileWriterDataPts = new FileWriter("/home/lvuser/Data/Log " + System.currentTimeMillis() + ".csv");
				fileWriterDataPts.append("Timestamp, xPos, yPos, avgEncoderDist, avgEncoderRate, elbowAngle, elbowRate, wristAngle, wristRate, gyroAngle, gyroRate, joystickDriver, joystickOperator, elbowCurrent, wristCurrent, elbowPwm, wristPwm, avgDriveTrainCurrent");
				fileWriterDataPts.append(NEW_LINE_SEPARATOR);
				for(DataPts p : savedDataPts) {
					fileWriterDataPts.append(String.valueOf(p.getTimeStamp()));
					fileWriterDataPts.append(COMMA_DELIMITER);
					for (double value : p.getDataPts()) {
						fileWriterDataPts.append(String.valueOf(value));
						fileWriterDataPts.append(COMMA_DELIMITER);

					}
					fileWriterDataPts.append(NEW_LINE_SEPARATOR);

				}


				fileWriterDataPts.flush();
				fileWriterDataPts.close();
			}

			savedEvents = new ArrayList<>();
			savedDataPts = new ArrayList<>();

		} catch (Exception e) {
			System.out.println("Error creating csvFileWriter");
			e.printStackTrace();
		} finally {

		}
	}
}
