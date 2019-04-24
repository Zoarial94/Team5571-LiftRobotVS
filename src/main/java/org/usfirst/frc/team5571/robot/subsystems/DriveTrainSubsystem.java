package org.usfirst.frc.team5571.robot.subsystems;

import org.usfirst.frc.team5571.robot.*;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import org.usfirst.frc.team5571.robot.commands.DriveTrain.DriveTrainDrive;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.PWMTalonSRX;;

public class DriveTrainSubsystem extends Subsystem {
	
	DifferentialDrive drive;
	
	public DriveTrainSubsystem() {
		
		drive = new DifferentialDrive(
    			new PWMTalonSRX(RobotMap.DRIVETRAIN_LEFTMOTOR),
    			new PWMTalonSRX(RobotMap.DRIVETRAIN_RIGHTMOTOR)
    			);
		
		//Check https://wpilib.screenstepslive.com/s/currentCS/m/java/l/599717-encoders-measuring-rotation-of-a-wheel-or-other-shaft
		//for info on encoders
	
		
	}
	
	public void arcadeDrive(double x, double y) {
		double sensitivity = Robot.m_driveTrainSensitivity;
    	drive.arcadeDrive(x * sensitivity, y * sensitivity);
    }

	public void initDefaultCommand() {
		setDefaultCommand(new DriveTrainDrive());
	}
}
