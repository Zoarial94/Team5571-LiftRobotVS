package org.usfirst.frc.team5571.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import org.usfirst.frc.team5571.robot.RobotMap;
import org.usfirst.frc.team5571.robot.commands.DriveTrain.DriveTrainDriveEncoders;
import org.usfirst.frc.team5571.robot.Constants;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 * Uses TalonSRX CAN devices to drive the drive train
 * Has multiple modes for testing and other use cases
 * 
 * Mode 0 - Disabled
 *  -- Does not use PID
 * Mode 1  - Arcade Drive
 * Mode 2  - Constant Velocity
 * Mode 3  - Set Distance
 *  -- Uses PID
 * Mode 11 - Arcade Drive
 * Mode 12 - Constant Velocity
 * Mode 13 - Set Distance
 * 
 * 
 * 
 * 
 */
public class DriveTrainEncoderSubsystem extends Subsystem {

	int       _driveMode;
	double    _sensitivity;
	TalonSRX  _rightMotor, _leftMotor;
	
	public DriveTrainEncoderSubsystem() {
		
		_sensitivity = 0.6;
		_driveMode = 1;

		_rightMotor = new TalonSRX(RobotMap.TESTING_TALON);
		_rightMotor.set(ControlMode.PercentOutput, 0);
		_rightMotor.configFactoryDefault();
		_rightMotor.setNeutralMode(NeutralMode.Brake);
		_rightMotor.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder,		// Local Feedback Source
													Constants.PID_PRIMARY,			// PID Slot for Source [0, 1]
													Constants.kTimeoutMs); 			// Configuration Timeout


		_leftMotor = new TalonSRX(RobotMap.TESTING_TALON);
		_leftMotor.set(ControlMode.PercentOutput, 0);
		_leftMotor.configFactoryDefault();
		_leftMotor.setNeutralMode(NeutralMode.Brake);
		_leftMotor.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder,		// Local Feedback Source
													Constants.PID_PRIMARY,			// PID Slot for Source [0, 1]
													Constants.kTimeoutMs); 			// Configuration Timeout
	}
	
	public void arcadeDrive(double speed, double rotation) {

		if(_driveMode == 0) {

			//Do nothing

		} else if(_driveMode == 1) {

			

		} else if(_driveMode == 2) {



		} else if(_driveMode == 3) {



		} else if(_driveMode == 11) {



		} else if(_driveMode == 12) {



		} else if(_driveMode == 13) {



		} else {



		}

		_rightMotor.set(ControlMode.PercentOutput, speed * _sensitivity);
    }
	
	public void setSensitivity(double sen) {
		if(sen > 1 || sen < -1) {
			_sensitivity = 1;
		} else {
			_sensitivity = Math.abs(sen);
		}
	}

	public boolean setMode(int mode) {
		this._driveMode = mode;
		return true;
	}

	public int getMode() {
		return _driveMode;
	}
	
	public double getSensitivity() {
		return _sensitivity;
	}

	public void initDefaultCommand() {
		setDefaultCommand(new DriveTrainDriveEncoders());
	}
}

