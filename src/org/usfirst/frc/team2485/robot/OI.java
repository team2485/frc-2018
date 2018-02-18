package org.usfirst.frc.team2485.robot;

import org.usfirst.frc.team2485.robot.commandGroups.Eject;
import org.usfirst.frc.team2485.robot.commands.ArmEmergencyControl;
import org.usfirst.frc.team2485.robot.commands.ArmSetSetpoint;
import org.usfirst.frc.team2485.robot.commands.HoldPosition;
import org.usfirst.frc.team2485.robot.commands.SetIntakeManual;
import org.usfirst.frc.team2485.robot.subsystems.Arm.ArmSetpoint;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
	public static final double XBOX_AXIS_DEADBAND = 0.2;
	public static final double XBOX_TRIGGER_DEADBAND = 0.02;
	
	public static Joystick driver;
	public static Joystick operator;
	
	public static final int XBOX_A_PORT = 1;
	public static final int XBOX_B_PORT = 2;
	public static final int XBOX_X_PORT = 3;
	public static final int XBOX_Y_PORT = 4;
	public static final int XBOX_LBUMPER_PORT = 5;
	public static final int XBOX_RBUMPER_PORT = 6;  
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
	
	
	public static void init() {
		driver = new Joystick(0);
		operator = new Joystick(1);
		
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

		

		DRIVER_B.whenPressed(new Eject());
		DRIVER_Y.whenPressed(new SetIntakeManual(0));
		DRIVER_A.whenPressed(new SetIntakeManual(1));
		OPERATOR_A.whenPressed(new ArmSetSetpoint(ArmSetpoint.INTAKE));
		OPERATOR_Y.whenPressed(new ArmSetSetpoint(ArmSetpoint.SCALE));
		OPERATOR_Y.whenPressed(new SetIntakeManual(0));
		OPERATOR_B.whenPressed(new ArmSetSetpoint(ArmSetpoint.SWITCH));
		OPERATOR_B.whenPressed(new SetIntakeManual(0));
		OPERATOR_X.whenPressed(new ArmSetSetpoint(ArmSetpoint.SCALE_BACKWARDS));
		OPERATOR_X.whenPressed(new SetIntakeManual(0));
		
		OPERATOR_A.whenReleased(new HoldPosition());
		OPERATOR_B.whenReleased(new HoldPosition());
		OPERATOR_X.whenReleased(new HoldPosition());
		OPERATOR_Y.whenReleased(new HoldPosition());

//		OPERATOR_LSTICK_BUTTON.whenPressed(new ArmEmergencyControl());
		
		
		
		
		

		//TESTING
//		DRIVER_X.whenPressed(new ZeroArmEncoders());
		
	}
	
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
}
