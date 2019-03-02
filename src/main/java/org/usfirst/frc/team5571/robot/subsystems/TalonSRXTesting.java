/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5571.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team5571.robot.RobotMap;
import org.usfirst.frc.team5571.robot.Constants;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 *
 * Uses TalonSRX CAN device(s)
 * Has multiple modes of operation
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
public class TalonSRXTesting extends Subsystem {
  
  int       _driveMode;
	double    _sensitivity;
  TalonSRX  _testMotor;
  boolean   _printDebug;
	
	public TalonSRXTesting() {
		
		_sensitivity = 0.6;
		_driveMode = 1;

		_testMotor = new TalonSRX(RobotMap.TESTING_TALON);
		_testMotor.set(ControlMode.PercentOutput, 0);
		_testMotor.configFactoryDefault();
    _testMotor.setNeutralMode(NeutralMode.Brake);

    _testMotor.setInverted(false);
    _testMotor.setSensorPhase(false);
    
		_testMotor.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder,		// Local Feedback Source
													                    Constants.PID_PRIMARY,			  // PID Slot for Source [0, 1]
                                              Constants.kTimeoutMs); 			  // Configuration Timeout
                                              
    

    
	}
	
	public void arcadeDrive(double speed, double rotation) {

		if(_driveMode == 0) {

			//Do nothing

		} else if(_driveMode == 1) {

			_testMotor.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, rotation);

		} else if(_driveMode == 2) {



		} else if(_driveMode == 3) {



		} else if(_driveMode == 11) {



		} else if(_driveMode == 12) {



		} else if(_driveMode == 13) {



		} else {



		}

			_testMotor.set(ControlMode.PercentOutput, speed * _sensitivity);
    }
	
	public void setSensitivity(double sen) {
		if(sen > 1 || sen < -1) {
			_sensitivity = 1;
		} else {
			_sensitivity = Math.abs(sen);
		}
	}

	public boolean setMode(int mode) {
		_driveMode = mode;
		return true;
	}

	public int getMode() {
		return _driveMode;
	}
	
	public double getSensitivity() {
		return _sensitivity;
  }
  
  public void setDebug(boolean debug) {
    _printDebug = debug;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
