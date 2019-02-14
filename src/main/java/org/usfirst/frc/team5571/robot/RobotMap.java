/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5571.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	/**
	 * NAMING
	 * 
	 * Naming should be structured like this in all caps:
	 *   [NameOfSystem]_[Location][Device]
	 * 
	 * Examples:
	 *    OUTPUT:
	 *    
	 *       public static final int DRIVETRAIN_LEFTMOTOR   =  0,
     * 								 DRIVETRAIN_RIGHTMOTOR  =  1,
     * 								 ELEVATOR_MOTOR         =  2;   (Has no location so just use 'MOTOR')
     *       
     *    INPUT:
     *    
     *       public static final int DRIVETRAIN_LEFTENCODER  = 0,
     *								 DRIVETRAIN_RIGHTENCODER = 1;
     *       
	 * 
	 */
	
	/*
	 * USB PORTS
	 * 1 for joystick/controller
	 */
	public static final int CONTROLLER = 0;
	
	/*
	 * PMW OUT
	 * 2 for wheels **WILL CHANGE TO CAN**
	 * 1 for pulley for elevator/carriage
	 * 2 for claw
	 */
						   	//Each motor output will control 2 motors in hardware
	public static final int DRIVETRAIN_LEFTMOTOR    = 0,
                            DRIVETRAIN_RIGHTMOTOR   = 1,
                            
                            //Lifts carriage and elevator
                            ELEVATOR_MOTOR          = 2,
                            
                            //Right and left side of the claw
                            CLAW_LEFTMOTOR          = 3,
                            CLAW_RIGHTMOTOR         = 4;
	
	/*
	 * CAN Communications
	 * 1 for testing
	 */
	
	public static final int DRIVETRAIN_RIGHTENCODMOTOR   = 0,
							DRIVETRAIN_LEFTENCODMOTOR    = 1;
	
	/*
	 * DIGITAL INPUT
	 * 2 for Elevator (Limit Switches)
	 * 2 encoders and 2 inputs for ONE rotary encoder (4 total)
	 */
	public static final int ELEVATOR_TOPSWITCH        = 0,
							ELEVATOR_BOTTOMSWITCH     = 1,
							
							//Left Encoder
							DRIVETRAIN_LEFTENCODER_A  = 2,
							DRIVETRAIN_LEFTENCODER_B  = 3,
							
							//Right Encoder
							DRIVETRAIN_RIGHTENCODER_A = 4,
							DRIVETRAIN_RIGHTENCODER_B = 5;
	/*
	 * ANALOG INPUT
	 * 2 for Claw
	 */
							//POT is short for potentiometer
	public static final int CLAW_LEFTPOT   = 0,
							CLAW_RIGHTPOT  = 1;
	
}
