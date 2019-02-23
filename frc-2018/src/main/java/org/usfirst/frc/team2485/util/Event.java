package org.usfirst.frc.team2485.util;

import java.io.FileWriter;

import java.io.IOException;

import java.util.ArrayList;

import java.util.List;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class Event {
	public enum Type {
		START, STOP, ERROR, DOUBLE;
	}
	private double timeStamp;
	private Type type;
	private String commandName;
	private String msg;
	
	public Event(double timeStamp, Type type, String commandName, String msg) {
		this.timeStamp = timeStamp;
		this.setType(type);
		this.setCommandName(commandName);
		this.setMsg(msg);
	}

	public Type getType() {
		return type;
	}

	public void setType(Type type) {
		this.type = type;
	}

	public String getCommandName() {
		return commandName;
	}

	public void setCommandName(String commandName) {
		this.commandName = commandName;
	}

	public String getMsg() {
		return msg;
	}

	public void setMsg(String msg) {
		this.msg = msg;
	}
	
	public double getTimeStamp() {
		return timeStamp;
	}
	
	public void setTimeStamp(double timeStamp) {
		this.timeStamp = timeStamp;
	}
	
	public String toString() {
		return "";
	}
//	public double getTimeStamp() {
//		return Timer.getFPGATimestamp();
//	}
//	public String setType() {
//		if(/* Command Initialized */) {
//			return /* Insert Command + */ "Started " + Timer.getFPGATimestamp() + " seconds after robot enable";
//		}
//		if(/* command calls isFinished */) {
//			return /*
//		}
//	}
	
	// Make Enum for Type (Start and Stop at least for now)
}

