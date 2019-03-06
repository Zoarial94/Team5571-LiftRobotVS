package org.usfirst.frc.team5571.robot.commands.Elevator;

import org.usfirst.frc.team5571.robot.Robot;
import org.usfirst.frc.team5571.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ElevatorLower extends Command {

	ElevatorSubsystem SUB;

    public ElevatorLower() {
    	SUB = Robot.m_ElevatorSub;
        requires(SUB);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if(SUB.canMoveDown()) {
    		System.out.println("Lowering Elevator");
			SUB.lowerElevator();
			return;
		}
    	System.out.println("NOT Lowering Elevator, switch it set");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(SUB.canMoveDown()) {
    		return false;
    	}
    	System.out.println("Finished Lowering Elevator up");
    	return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    	SUB.stopElevator();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("Elevator Raising Inturrupted");
    	end();
    }
}
