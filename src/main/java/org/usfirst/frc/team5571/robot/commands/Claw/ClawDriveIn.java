package org.usfirst.frc.team5571.robot.commands.Claw;
import org.usfirst.frc.team5571.robot.Robot;
import org.usfirst.frc.team5571.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj.command.Command;
/*
 * use this command to make the claw drive in the conveyer belt.
 */
public class ClawDriveIn extends Command
{
	ClawSubsystem subsystem;
	public ClawDriveIn()
	{
		subsystem=Robot.m_ClawSub;
		requires(subsystem);
	}
	public void initialize()
	{
		subsystem.driveIn();
	}
	public void execute()
	{
		
	}
	public void end()
	{
		subsystem.stopDrive();
	}
	public boolean isFinished()
	{
		return(false);
	}
	public void interrupted()
	{
		end();
	}
}