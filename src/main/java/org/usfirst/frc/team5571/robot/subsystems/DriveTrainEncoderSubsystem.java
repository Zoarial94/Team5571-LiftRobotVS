package org.usfirst.frc.team5571.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import org.usfirst.frc.team5571.robot.RobotMap;
import org.usfirst.frc.team5571.robot.commands.DriveTrainDriveEncoders;
import org.usfirst.frc.team5571.robot.Constants ;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class DriveTrainEncoderSubsystem extends Subsystem {
	
	int mode;
	double sensitivity;
	TalonSRX _testMotor;
	
	public DriveTrainEncoderSubsystem() {
		
		sensitivity = 0.6;
		mode = 1;

		_testMotor = new TalonSRX(20);
		_testMotor.set(ControlMode.PercentOutput, 0);
		_testMotor.configFactoryDefault();
		_testMotor.setNeutralMode(NeutralMode.Brake);
		_testMotor.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder,		// Local Feedback Source
													Constants.PID_PRIMARY,			// PID Slot for Source [0, 1]
													Constants.kTimeoutMs); 			// Configuration Timeout
	}
	
	public void arcadeDrive(double x, double y) {
			_testMotor.set(ControlMode.PercentOutput, x * sensitivity);
    }
	
	public void setSensitivity(double sen) {
		if(sen > 1 || sen < -1) {
			sensitivity = 1;
		} else {
			sensitivity = Math.abs(sen);
		}
	}

	/*
	 * Set mode for driving
	 * Mode 0 - Off
	 * Mode 1 - Arcade Drive
	 * Mode 2 - 
	 */
	public boolean setMode(int mode) {
		this.mode = mode;
		return true;
	}

	public int getMode() {
		return mode;
	}
	
	public double getSensitivity() {
		return sensitivity;
	}

	public void initDefaultCommand() {
		setDefaultCommand(new DriveTrainDriveEncoders());
	}
}

