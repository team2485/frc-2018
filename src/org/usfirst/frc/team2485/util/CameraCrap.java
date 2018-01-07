package org.usfirst.frc.team2485.util;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;

/**
 * @author Ben Dorsey
 */

public class CameraCrap {
	private static UsbCamera usbCamera;
	public static void init() {
		usbCamera = new UsbCamera("USB Camera 0", 0); // get camera

		MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181);

		mjpegServer1.setSource(usbCamera); 
	}
}
