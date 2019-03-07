package org.usfirst.frc.team5571.robot.commands.DriveTrain;

import org.usfirst.frc.team5571.robot.Robot;
import org.usfirst.frc.team5571.robot.Constants;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class SetNormalSpeed extends InstantCommand {

    public SetNormalSpeed() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    	Robot.m_driveTrainSensitivity = Constants.driveTrainNormalSpeed;
    }

}
