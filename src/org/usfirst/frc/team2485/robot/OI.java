package org.usfirst.frc.team2485.robot;

import org.usfirst.frc.team2485.robot.commands.IntakeWithControllers;
import org.usfirst.frc.team2485.robot.commands.ZeroArmEncoders;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
	public static final double XBOX_DEADBAND = 0.25;
	
	public static Joystick DRIVER;
	public static Joystick OPERATOR;
	
	public static final int XBOX_A_PORT = 1;
	public static final int XBOX_B_PORT = 2;
	public static final int XBOX_X_PORT = 3;
	public static final int XBOX_Y_PORT = 4;
	public static final int XBOX_LBUMPER_PORT = 5;
	public static final int XBOX_RBUMPER_PORT = 6;  
	public static final int XBOX_START_PORT = 9;  
	public static final int XBOX_BACK_PORT = 10;  
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
	
	
	public static void init() {
		DRIVER = new Joystick(0);
		OPERATOR = new Joystick(1);
		
		DRIVER_UP = new JoystickButton(DRIVER, XBOX_UP_PORT);
		DRIVER_DOWN = new JoystickButton(DRIVER, XBOX_DOWN_PORT);
		DRIVER_LEFT = new JoystickButton(DRIVER, XBOX_LEFT_PORT);
		DRIVER_RIGHT = new JoystickButton(DRIVER, XBOX_RIGHT_PORT);
		
		DRIVER_A = new JoystickButton(DRIVER, XBOX_A_PORT);
		DRIVER_B = new JoystickButton(DRIVER, XBOX_B_PORT);
		DRIVER_X = new JoystickButton(DRIVER, XBOX_X_PORT);
		DRIVER_Y = new JoystickButton(DRIVER, XBOX_Y_PORT);
		
		DRIVER_LBUMPER = new JoystickButton(DRIVER, XBOX_LBUMPER_PORT);
		DRIVER_RBUMPER = new JoystickButton(DRIVER, XBOX_RBUMPER_PORT);
		
		DRIVER_XBOX = new JoystickButton(DRIVER, XBOX_XBOX_PORT);
		
		OPERATOR_UP = new JoystickButton(DRIVER, XBOX_UP_PORT);
		OPERATOR_DOWN = new JoystickButton(DRIVER, XBOX_DOWN_PORT);
		OPERATOR_LEFT = new JoystickButton(DRIVER, XBOX_LEFT_PORT);
		OPERATOR_RIGHT = new JoystickButton(DRIVER, XBOX_RIGHT_PORT);
		
		OPERATOR_A = new JoystickButton(DRIVER, XBOX_A_PORT);
		OPERATOR_B = new JoystickButton(DRIVER, XBOX_B_PORT);
		OPERATOR_X = new JoystickButton(DRIVER, XBOX_X_PORT);
		OPERATOR_Y = new JoystickButton(DRIVER, XBOX_Y_PORT);
		
		OPERATOR_LBUMPER = new JoystickButton(DRIVER, XBOX_LBUMPER_PORT);
		OPERATOR_RBUMPER = new JoystickButton(DRIVER, XBOX_RBUMPER_PORT);
		
		OPERATOR_XBOX = new JoystickButton(DRIVER, XBOX_XBOX_PORT);
		
		DRIVER_A.whenPressed(new IntakeWithControllers(0.8));
		DRIVER_B.whenPressed(new IntakeWithControllers(0));
		
		//TESTING
		DRIVER_X.whenPressed(new ZeroArmEncoders());
		
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
