/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5571.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team5571.robot.*;
import org.usfirst.frc.team5571.robot.subsystems.*;


public class TalonSRXCommand extends Command {

  public static TalonSRXTesting SUB;

  public TalonSRXCommand() {
    SUB = Robot.m_TalonSRX;
    requires(SUB);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double sen = SUB.getSensitivity();
    SUB.arcadeDrive(
    	Robot.m_oi.controller.getY() * sen, 
    	Robot.m_oi.controller.getX() * sen 
    );
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
