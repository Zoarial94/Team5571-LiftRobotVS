package org.usfirst.frc.team5571.robot.commands.DriveTrain;

import org.usfirst.frc.team5571.robot.Robot;
import org.usfirst.frc.team5571.robot.Constants;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetNormalSpeed extends Command {

    public SetNormalSpeed() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.m_driveTrainSensitivity = Constants.driveTrainNormalSpeed;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
