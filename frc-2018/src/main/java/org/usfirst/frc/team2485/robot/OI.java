package org.usfirst.frc.team2485.robot;

import org.usfirst.frc.team2485.robot.commandGroups.ArmToScale;
import org.usfirst.frc.team2485.robot.commandGroups.Eject;
import org.usfirst.frc.team2485.robot.commands.ArmEmergencyControl;
import org.usfirst.frc.team2485.robot.commands.ArmSetSetpoint;
import org.usfirst.frc.team2485.robot.commands.CancelCommand;
import org.usfirst.frc.team2485.robot.commands.HoldPosition;
import org.usfirst.frc.team2485.robot.commands.SetIntakeManual;
import org.usfirst.frc.team2485.robot.commands.StartEjecting;
import org.usfirst.frc.team2485.robot.commands.StopIntaking;
import org.usfirst.frc.team2485.robot.commands.G13Testing;
import org.usfirst.frc.team2485.robot.subsystems.Arm.ArmSetpoint;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
	public static final double XBOX_AXIS_DEADBAND = 0.1;
	public static final double XBOX_TRIGGER_DEADBAND = 0.02;
	
	public static Joystick driver;
	public static Joystick operator;
	public static Joystick driverBackup;
	public static Joystick operatorBackup;
	
	public static final int XBOX_A_PORT = 1;
	public static final int XBOX_B_PORT = 2;
	public static final int XBOX_X_PORT = 3;
	public static final int XBOX_Y_PORT = 4;
	public static final int XBOX_LBUMPER_PORT = 5;
	public static final int XBOX_RBUMPER_PORT = 6; 
	public static final int XBOX_BACK_BUTTON = 7;
	public static final int XBOX_START_BUTTON = 8;
	public static final int XBOX_LSTICK_BUTTON_PORT = 9;  
	public static final int XBOX_RSTICK_BUTTON_PORT = 10;  
	public static final int XBOX_XBOX_PORT = 11;  
	public static final int XBOX_UP_PORT = 12;  
	public static final int XBOX_DOWN_PORT = 13;  
	public static final int XBOX_LEFT_PORT = 14;
	public static final int XBOX_RIGHT_PORT = 15;
	
	public static final int XBOX_LXJOSYSTICK_PORT = 0;
	public static final int XBOX_LYJOYSTICK_PORT = 1;
	public static final int XBOX_LTRIGGER_PORT = 2;
	public static final int XBOX_RTRIGGER_PORT = 3;
	public static final int XBOX_RXJOYSTICK_PORT = 4;
	public static final int XBOX_RYJOYSTICK_PORT = 5;

	
	public static JoystickButton DRIVER_UP;
	public static JoystickButton DRIVER_DOWN;
	public static JoystickButton DRIVER_LEFT;
	public static JoystickButton DRIVER_RIGHT;
	public static JoystickButton DRIVER_A;
	public static JoystickButton DRIVER_B;
	public static JoystickButton DRIVER_X;
	public static JoystickButton DRIVER_Y;
	public static JoystickButton DRIVER_LBUMPER;
	public static JoystickButton DRIVER_RBUMPER;
	public static JoystickButton DRIVER_XBOX;
	public static JoystickButton DRIVER_START;
	
	public static JoystickButton OPERATOR_UP;
	public static JoystickButton OPERATOR_DOWN;
	public static JoystickButton OPERATOR_LEFT;
	public static JoystickButton OPERATOR_RIGHT;
	public static JoystickButton OPERATOR_A;
	public static JoystickButton OPERATOR_B;
	public static JoystickButton OPERATOR_X;
	public static JoystickButton OPERATOR_Y;
	public static JoystickButton OPERATOR_LBUMPER;
	public static JoystickButton OPERATOR_RBUMPER;
	public static JoystickButton OPERATOR_XBOX;
	public static JoystickButton OPERATOR_LSTICK_BUTTON;
	public static JoystickButton OPERATOR_RSTICK_BUTTON;
	public static JoystickButton OPERATOR_START_BUTTON;
	public static JoystickButton OPERATOR_BACK_BUTTON;
	
	
//Backup Buttons
	public static JoystickButton DRIVER_BACKUP_UP;
	public static JoystickButton DRIVER_BACKUP_DOWN;
	public static JoystickButton DRIVER_BACKUP_LEFT;
	private static JoystickButton DRIVER_BACKUP_RIGHT;
	private static JoystickButton DRIVER_BACKUP_A;
	private static JoystickButton DRIVER_BACKUP_B;
	private static JoystickButton DRIVER_BACKUP_X;
	private static JoystickButton DRIVER_BACKUP_Y;
	private static JoystickButton DRIVER_BACKUP_LBUMPER;
	private static JoystickButton DRIVER_BACKUP_RBUMPER;
	private static JoystickButton DRIVER_BACKUP_XBOX;
	
	private static JoystickButton OPERATOR_BACKUP_UP;
	private static JoystickButton OPERATOR_BACKUP_DOWN;
	private static JoystickButton OPERATOR_BACKUP_LEFT;
	private static JoystickButton OPERATOR_BACKUP_RIGHT;
	private static JoystickButton OPERATOR_BACKUP_A;
	private static JoystickButton OPERATOR_BACKUP_B;
	private static JoystickButton OPERATOR_BACKUP_X;
	private static JoystickButton OPERATOR_BACKUP_Y;
	private static JoystickButton OPERATOR_BACKUP_LBUMPER;
	private static JoystickButton OPERATOR_BACKUP_RBUMPER;
	private static JoystickButton OPERATOR_BACKUP_XBOX;
	private static JoystickButton OPERATOR_BACKUP_LSTICK_BUTTON;
	private static JoystickButton OPERATOR_BACKUP_RSTICK_BUTTON;
	private static JoystickButton OPERATOR_BACKUP_START_BUTTON;
	private static JoystickButton OPERATOR_BACKUP_BACK_BUTTON;
	private static JoystickButton DRIVER_RSTICK_BUTTON;
	
	
	public static Joystick hell;

	public static final int G13_BUTTON_PORT1 = 0;
	public static final int G13_BUTTON_PORT2 = 1;

	public static JoystickButton G13_Button1;
	public static JoystickButton G13_Button2;

	public static void init() {
		driver = new Joystick(0);
		operator = new Joystick(1);
		driverBackup = new Joystick(2);
		operatorBackup = new Joystick(3);
		
		DRIVER_UP = new JoystickButton(driver, XBOX_UP_PORT);
		DRIVER_DOWN = new JoystickButton(driver, XBOX_DOWN_PORT);
		DRIVER_LEFT = new JoystickButton(driver, XBOX_LEFT_PORT);
		DRIVER_RIGHT = new JoystickButton(driver, XBOX_RIGHT_PORT);
		
		DRIVER_A = new JoystickButton(driver, XBOX_A_PORT);
		DRIVER_B = new JoystickButton(driver, XBOX_B_PORT);
		DRIVER_X = new JoystickButton(driver, XBOX_X_PORT);
		DRIVER_Y = new JoystickButton(driver, XBOX_Y_PORT);
		
		DRIVER_LBUMPER = new JoystickButton(driver, XBOX_LBUMPER_PORT);
		DRIVER_RBUMPER = new JoystickButton(driver, XBOX_RBUMPER_PORT);
		
		DRIVER_XBOX = new JoystickButton(driver, XBOX_XBOX_PORT);
		DRIVER_START = new JoystickButton(driver, XBOX_START_BUTTON);
		
		OPERATOR_UP = new JoystickButton(operator, XBOX_UP_PORT);
		OPERATOR_DOWN = new JoystickButton(operator, XBOX_DOWN_PORT);
		OPERATOR_LEFT = new JoystickButton(operator, XBOX_LEFT_PORT);
		OPERATOR_RIGHT = new JoystickButton(operator, XBOX_RIGHT_PORT);
		
		OPERATOR_A = new JoystickButton(operator, XBOX_A_PORT);
		OPERATOR_B = new JoystickButton(operator, XBOX_B_PORT);
		OPERATOR_X = new JoystickButton(operator, XBOX_X_PORT);
		OPERATOR_Y = new JoystickButton(operator, XBOX_Y_PORT);
		
		OPERATOR_LBUMPER = new JoystickButton(operator, XBOX_LBUMPER_PORT);
		OPERATOR_RBUMPER = new JoystickButton(operator, XBOX_RBUMPER_PORT);
		
		OPERATOR_XBOX = new JoystickButton(operator, XBOX_XBOX_PORT);
		
		OPERATOR_LSTICK_BUTTON = new JoystickButton(operator, XBOX_LSTICK_BUTTON_PORT);
		OPERATOR_RSTICK_BUTTON = new JoystickButton(operator, XBOX_RSTICK_BUTTON_PORT);
		
		OPERATOR_START_BUTTON = new JoystickButton(operator, XBOX_START_BUTTON);
		OPERATOR_BACK_BUTTON = new JoystickButton(operator, XBOX_BACK_BUTTON);
		
		DRIVER_RSTICK_BUTTON = new JoystickButton(driver, XBOX_RSTICK_BUTTON_PORT);


		
		
		DRIVER_START.whenPressed(new StartEjecting(false, true));
		DRIVER_B.whenPressed(new Eject(true, false, false));
		DRIVER_RBUMPER.whenPressed(new Eject(false, false, false));
		DRIVER_Y.whenPressed(new SetIntakeManual(0));
		DRIVER_A.whenPressed(new SetIntakeManual(.75)); // anton change this 
		DRIVER_RSTICK_BUTTON.whenPressed(new SetIntakeManual(.5));
		
		Command scaleHighBack = new ArmToScale(ArmSetpoint.SCALE_HIGH_BACK);
		OPERATOR_Y.whenPressed(scaleHighBack);
		OPERATOR_Y.whenReleased(new CancelCommand(scaleHighBack));
		OPERATOR_Y.whenPressed(new StopIntaking()); // only stops if intake rollers running forward
		
		Command scaleMiddleBack = new ArmToScale(ArmSetpoint.SCALE_MIDDLE_BACK);
		OPERATOR_X.whenPressed(scaleMiddleBack);
		OPERATOR_X.whenReleased(new CancelCommand(scaleMiddleBack));
		OPERATOR_X.whenPressed(new StopIntaking());
		
		OPERATOR_B.whenPressed(new ArmSetSetpoint(ArmSetpoint.SWITCH));
		OPERATOR_B.whenPressed(new StopIntaking());
		
		Command scaleLowBack = new ArmToScale(ArmSetpoint.SCALE_LOW_BACK);
		OPERATOR_A.whenPressed(scaleLowBack);
		OPERATOR_A.whenReleased(new CancelCommand(scaleLowBack));
		OPERATOR_A.whenPressed(new StopIntaking());
		
		OPERATOR_RBUMPER.whenPressed(new ArmSetSetpoint(ArmSetpoint.SECOND_STORY));
		OPERATOR_LBUMPER.whenPressed(new ArmSetSetpoint(ArmSetpoint.INTAKE));
		
		Command scaleSeven = new ArmToScale(ArmSetpoint.SEVEN_FOOT_SCALE);
		OPERATOR_START_BUTTON.whenPressed(scaleSeven);	
		OPERATOR_START_BUTTON.whenReleased(new CancelCommand(scaleSeven));
		OPERATOR_START_BUTTON.whenPressed(new StopIntaking());
		
		Command climber = new ArmToScale(ArmSetpoint.CLIMB);
		OPERATOR_BACK_BUTTON.whenPressed(climber);
		OPERATOR_BACK_BUTTON.whenReleased(new CancelCommand(climber));
		OPERATOR_BACK_BUTTON.whenPressed(new StopIntaking());
	

//		OPERATOR_A.whenReleased(new HoldPosition());
//		OPERATOR_B.whenReleased(new HoldPosition());
//		OPERATOR_X.whenReleased(new HoldPosition());
//		OPERATOR_Y.whenReleased(new HoldPosition());
//		OPERATOR_RBUMPER.whenReleased(new HoldPosition());
//		OPERATOR_LBUMPER.whenReleased(new HoldPosition());
		Command c = new ArmEmergencyControl();
		OPERATOR_LSTICK_BUTTON.whenPressed(c);
		OPERATOR_RSTICK_BUTTON.cancelWhenPressed(c);
//		OPERATOR_START_BUTTON.whenReleased(new HoldPosition());
//		OPERATOR_BACK_BUTTON.whenReleased(new HoldPosition());
		
		
		
		
		

		//TESTING
//		DRIVER_X.whenPressed(new ZeroArmEncoders());
		
		DRIVER_BACKUP_UP = new JoystickButton(driverBackup, XBOX_UP_PORT);
		DRIVER_BACKUP_DOWN = new JoystickButton(driverBackup, XBOX_DOWN_PORT);
		DRIVER_BACKUP_LEFT = new JoystickButton(driverBackup, XBOX_LEFT_PORT);
		DRIVER_BACKUP_RIGHT = new JoystickButton(driverBackup, XBOX_RIGHT_PORT);
		
		DRIVER_BACKUP_A = new JoystickButton(driverBackup, XBOX_A_PORT);
		DRIVER_BACKUP_B = new JoystickButton(driverBackup, XBOX_B_PORT);
		DRIVER_BACKUP_X = new JoystickButton(driverBackup, XBOX_X_PORT);
		DRIVER_BACKUP_Y = new JoystickButton(driverBackup, XBOX_Y_PORT);
		
		DRIVER_BACKUP_LBUMPER = new JoystickButton(driverBackup, XBOX_LBUMPER_PORT);
		DRIVER_BACKUP_RBUMPER = new JoystickButton(driverBackup, XBOX_RBUMPER_PORT);
		
		DRIVER_BACKUP_XBOX = new JoystickButton(driverBackup, XBOX_XBOX_PORT);
		
		DRIVER_BACKUP_B.whenPressed(new Eject(true, false, false));
		DRIVER_BACKUP_RBUMPER.whenPressed(new Eject(false, false, false));
		DRIVER_BACKUP_Y.whenPressed(new SetIntakeManual(0));
		DRIVER_BACKUP_A.whenPressed(new SetIntakeManual(.75)); // anton change this 
		
		OPERATOR_BACKUP_UP = new JoystickButton(operatorBackup, XBOX_UP_PORT);
		OPERATOR_BACKUP_DOWN = new JoystickButton(operatorBackup, XBOX_DOWN_PORT);
		OPERATOR_BACKUP_LEFT = new JoystickButton(operatorBackup, XBOX_LEFT_PORT);
		OPERATOR_BACKUP_RIGHT = new JoystickButton(operatorBackup, XBOX_RIGHT_PORT);
		
		OPERATOR_BACKUP_A = new JoystickButton(operatorBackup, XBOX_A_PORT);
		OPERATOR_BACKUP_B = new JoystickButton(operatorBackup, XBOX_B_PORT);
		OPERATOR_BACKUP_X = new JoystickButton(operatorBackup, XBOX_X_PORT);
		OPERATOR_BACKUP_Y = new JoystickButton(operatorBackup, XBOX_Y_PORT);
		
		OPERATOR_BACKUP_LBUMPER = new JoystickButton(operatorBackup, XBOX_LBUMPER_PORT);
		OPERATOR_BACKUP_RBUMPER = new JoystickButton(operatorBackup, XBOX_RBUMPER_PORT);
		
		OPERATOR_BACKUP_XBOX = new JoystickButton(operatorBackup, XBOX_XBOX_PORT);
		
		OPERATOR_BACKUP_LSTICK_BUTTON = new JoystickButton(operatorBackup, XBOX_LSTICK_BUTTON_PORT);
		OPERATOR_BACKUP_RSTICK_BUTTON = new JoystickButton(operatorBackup, XBOX_RSTICK_BUTTON_PORT);
		
		OPERATOR_BACKUP_START_BUTTON = new JoystickButton(operatorBackup, XBOX_START_BUTTON);
		OPERATOR_BACKUP_BACK_BUTTON = new JoystickButton(operatorBackup, XBOX_BACK_BUTTON);
		
		
		OPERATOR_BACKUP_Y.whenPressed(scaleHighBack);
		OPERATOR_BACKUP_Y.whenReleased(new CancelCommand(scaleHighBack));
		OPERATOR_BACKUP_Y.whenPressed(new StopIntaking()); // only stops if intake rollers running forward
		
		OPERATOR_BACKUP_X.whenPressed(scaleMiddleBack);
		OPERATOR_BACKUP_X.whenReleased(new CancelCommand(scaleMiddleBack));
		OPERATOR_BACKUP_X.whenPressed(new StopIntaking());
		
		OPERATOR_BACKUP_B.whenPressed(new ArmSetSetpoint(ArmSetpoint.SWITCH));
		OPERATOR_BACKUP_B.whenPressed(new StopIntaking());
		
		OPERATOR_BACKUP_A.whenPressed(scaleLowBack);
		OPERATOR_BACKUP_A.whenReleased(new CancelCommand(scaleLowBack));
		OPERATOR_BACKUP_A.whenPressed(new StopIntaking());
		
		OPERATOR_BACKUP_RBUMPER.whenPressed(new ArmSetSetpoint(ArmSetpoint.SECOND_STORY));
		OPERATOR_BACKUP_LBUMPER.whenPressed(new ArmSetSetpoint(ArmSetpoint.INTAKE));
		
		OPERATOR_BACKUP_START_BUTTON.whenPressed(scaleSeven);	
		OPERATOR_BACKUP_START_BUTTON.whenReleased(new CancelCommand(scaleSeven));
		OPERATOR_BACKUP_START_BUTTON.whenPressed(new StopIntaking());
		
		OPERATOR_BACKUP_BACK_BUTTON.whenPressed(climber);
		OPERATOR_BACKUP_BACK_BUTTON.whenReleased(new CancelCommand(climber));
		OPERATOR_BACKUP_BACK_BUTTON.whenPressed(new StopIntaking());
	

		OPERATOR_BACKUP_A.whenReleased(new HoldPosition());
		OPERATOR_BACKUP_B.whenReleased(new HoldPosition());
		OPERATOR_BACKUP_X.whenReleased(new HoldPosition());
		OPERATOR_BACKUP_Y.whenReleased(new HoldPosition());
		OPERATOR_BACKUP_RBUMPER.whenReleased(new HoldPosition());
		OPERATOR_BACKUP_LBUMPER.whenReleased(new HoldPosition());
		OPERATOR_BACKUP_LSTICK_BUTTON.whenPressed(c);
		OPERATOR_BACKUP_RSTICK_BUTTON.cancelWhenPressed(c);
		OPERATOR_BACKUP_START_BUTTON.whenReleased(new HoldPosition());
		OPERATOR_BACKUP_BACK_BUTTON.whenReleased(new HoldPosition());

		hell = new Joystick(2);

		G13_Button1 = new JoystickButton(hell, G13_BUTTON_PORT2);
		G13_Button2 = new JoystickButton(hell, G13_BUTTON_PORT2);

		G13_Button1.whenPressed(new G13Testing());
		G13_Button2.whenPressed(new G13Testing());

	}
}
