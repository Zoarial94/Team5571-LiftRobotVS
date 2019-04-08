/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5571.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
	public XboxController controller = new XboxController(RobotMap.CONTROLLER);
	
	public JoystickButton LB = new JoystickButton(controller, 5),
				  		  RB = new JoystickButton(controller, 6),
						  X = new JoystickButton(controller, 3),
						  B = new JoystickButton(controller, 4),
						  SIDE = new JoystickButton(controller, 2),
						  BRAKE = new JoystickButton(controller, 5),
						  COAST = new JoystickButton(controller, 6);
}
