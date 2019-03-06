/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5571.robot.commands.Debugging;

import edu.wpi.first.wpilibj.command.InstantCommand;
import org.usfirst.frc.team5571.robot.Robot;

/**
 * Add your docs here.
 */
public class EnableMotorDebugging extends InstantCommand {
  /**
   * Add your docs here.
   */
  public EnableMotorDebugging() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.m_TalonSRX.setDebug(true);
  }

}