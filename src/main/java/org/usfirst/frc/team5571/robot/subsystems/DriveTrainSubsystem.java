package org.usfirst.frc.team5571.robot.subsystems;

import org.usfirst.frc.team5571.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import org.usfirst.frc.team5571.robot.commands.*;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.PWMTalonSRX;;

public class DriveTrainSubsystem extends Subsystem {
	
	DifferentialDrive drive;
	
	Encoder leftEncoder, rightEncoder;
	
	double sensitivity;
	
	public DriveTrainSubsystem() {
		
		drive = new DifferentialDrive(
    			new Spark(RobotMap.DRIVETRAIN_LEFTMOTOR),
    			new Spark(RobotMap.DRIVETRAIN_RIGHTMOTOR)
    			);
		
		//Check https://wpilib.screenstepslive.com/s/currentCS/m/java/l/599717-encoders-measuring-rotation-of-a-wheel-or-other-shaft
		//for info on encoders
		
		leftEncoder = new Encoder(RobotMap.DRIVETRAIN_LEFTENCODER_A,  RobotMap.DRIVETRAIN_LEFTENCODER_B,
								  false, Encoder.EncodingType.k4X); //Initialize Encoder
		
		leftEncoder.setMaxPeriod(.1);           //Time in seconds before timeout
		leftEncoder.setMinRate(10);   			//Rate considered stopped
		leftEncoder.setDistancePerPulse(5);		//Distance per pulse, based off of gear ratios
		leftEncoder.setReverseDirection(true); 	
		leftEncoder.setSamplesToAverage(7);		//???
		
		rightEncoder = new Encoder(RobotMap.DRIVETRAIN_RIGHTENCODER_A,  RobotMap.DRIVETRAIN_RIGHTENCODER_B,
								   false, Encoder.EncodingType.k4X);
		
		rightEncoder.setMaxPeriod(.1); 
		rightEncoder.setMinRate(10);
		rightEncoder.setDistancePerPulse(5);
		rightEncoder.setReverseDirection(false);
		rightEncoder.setSamplesToAverage(7);
		
		sensitivity = 0.6;
		
	}
	
	public double getDistanceLeft() {
		return leftEncoder.getDistance();
	}
	
	public double getDistanceRight() {
		return rightEncoder.getDistance();
	}
	
	public void arcadeDrive(double x, double y) {
    	drive.arcadeDrive(x * sensitivity, y * sensitivity);
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
		setDefaultCommand(new DriveTrainDrive());
	}
}
