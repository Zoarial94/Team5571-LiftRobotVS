package org.usfirst.frc.team5571.robot.commands.DriveTrain;

import org.usfirst.frc.team5571.robot.Robot;
import org.usfirst.frc.team5571.robot.subsystems.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import edu.wpi.first.wpilibj.command.Command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTrainDrive extends Command {
	
	boolean moving;
	
	DriveTrainSubsystem SUB;
	
    public DriveTrainDrive() {
    	SUB = Robot.m_DriveTrainSub;
    	requires(SUB);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.m_DriveTrainSub.arcadeDrive(
    			Robot.m_oi.controller.getY(Hand.kLeft), 
    			Robot.m_oi.controller.getX(Hand.kLeft)
    			);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
