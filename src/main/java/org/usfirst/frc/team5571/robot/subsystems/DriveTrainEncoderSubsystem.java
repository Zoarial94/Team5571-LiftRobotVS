package org.usfirst.frc.team5571.robot.subsystems;

import org.usfirst.frc.team5571.robot.RobotMap;
import org.usfirst.frc.team5571.robot.commands.DriveTrainDrive;
import org.usfirst.frc.team5571.robot.commands.DriveTrainDriveEncoders;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 *
 */
public class DriveTrainEncoderSubsystem extends Subsystem {

DifferentialDrive drive;
	
	double sensitivity;
	
	public DriveTrainEncoderSubsystem() {
		
		//Setup CAN devices
		//Try using drive = new DifferentialDrive()
		
		sensitivity = 0.6;
		
	}
	
	public void arcadeDrive(double x, double y) {
    	//drive.arcadeDrive(x * sensitivity, y * sensitivity);
    }
	
	public void setSensitivity(double sen) {
		if(sen > 1 || sen < -1) {
			sensitivity = 1;
		} else {
			sensitivity = Math.abs(sen);
		}
	}
	
	public double getSensitivity() {
		return sensitivity;
	}

	public void initDefaultCommand() {
		setDefaultCommand(new DriveTrainDriveEncoders());
	}
}

