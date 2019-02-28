package org.usfirst.frc.team5571.robot.subsystems;
import org.usfirst.frc.team5571.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.PWMTalonSRX;
/**
 *this subsystem is used to control the motors on the claw
 *
 *methods:
 *setSpeed(double): sets the speed for the motors to spin at.
 *getSpeed(): returns the current speed set for the motors to spin at.
 *driveIn(): sets the motors to drive the conveyor inwards.
 *driveOut(): sets the motors to drive the conveyor outwards.
 *driveStop(): stops any movement of the motors;
 */
public class ClawSubsystem extends Subsystem
{
	PWMTalonSRX LeftMotor;
	PWMTalonSRX RightMotor;
	double Speed;
    public ClawSubsystem()
    {
    	LeftMotor=new PWMTalonSRX(RobotMap.CLAW_LEFTMOTOR);
    	RightMotor=new PWMTalonSRX(RobotMap.CLAW_RIGHTMOTOR);
    	Speed=0.1;
    }
    public void setSpeed(double speed)
    {
    	this.Speed=Math.abs(speed);
    }
    public double getSpeed()
    {
    	return(Speed);
    }
    public void driveIn()
    {
    	LeftMotor.set(Speed);
    	RightMotor.set(-Speed);
    }
    public void driveOut()
    {
    	LeftMotor.set(-Speed);
    	RightMotor.set(Speed);
    }
    public void stopDrive()
    {
    	LeftMotor.set(0.0);
    	RightMotor.set(0.0);
    }
    public void initDefaultCommand()
    {
    	
    }
}

