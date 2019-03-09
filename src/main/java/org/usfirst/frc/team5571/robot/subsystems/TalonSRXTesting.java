/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5571.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team5571.robot.RobotMap;
import org.usfirst.frc.team5571.robot.Robot;
import org.usfirst.frc.team5571.robot.Constants;
import org.usfirst.frc.team5571.robot.commands.*;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

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
	double    _sensitivity, _lastSpeed, _lastTurn;
  TalonSRX  _testMotor;

  Faults _faults;
	
	public TalonSRXTesting() {
		
		_sensitivity = 0.6;
    _driveMode = 0;
    
    //Faults from one Talon Motor
    //Unsure of exact purpose
    _faults = new Faults();

    //Get can motor
    _testMotor = new TalonSRX(RobotMap.TESTING_TALON);
    //Output to zero for safety
    _testMotor.set(ControlMode.PercentOutput, 0);
    //Reset to defaults
    _testMotor.configFactoryDefault();
    //Brake/Coast when output is zero
    _testMotor.setNeutralMode(NeutralMode.Brake); //NeutralMode.Coast

    //Invert output
    _testMotor.setInverted(false);
    //Invert sensor output
    _testMotor.setSensorPhase(false);
    
    //THERE ARE TWO PID LOOPS: PRIMARY AND AUXILARY

    //Set local quad as primary feedback
		_testMotor.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder,		// Local Feedback Source
													                    Constants.PID_PRIMARY,			  // PID Slot for Source [0, 1]
                                              Constants.kTimeoutMs); 			  // Configuration Timeout
    
    //There are two SensorTerms: Sum and Diff
    //Sum will equal Sum0 + Sum1
    //Diff will equal Diff0 - Diff1

    //Set local quad as Sum0
    //Will be used as "Forward Distance"
    _testMotor.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs);
    //_otherMotor.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs);
    
    //Set local quad as Diff0
    //Will be used as "Turning"
    //Diff1 should be a remote sensor and Diff will show the difference between the left and right forward movement
    _testMotor.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs); 
    _testMotor.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs); 

    //Use Sum for the primary PID
    _testMotor.configSelectedFeedbackSensor(	FeedbackDevice.SensorSum, 
													                    Constants.PID_PRIMARY,
                                              Constants.kTimeoutMs);

    //Coefficient should be 0.5 if using two encoders for Sum
    _testMotor.configSelectedFeedbackCoefficient(	1, 						              // Coefficient
                                                  Constants.PID_PRIMARY,		// PID Slot of Source 
                                                  Constants.kTimeoutMs); 
    
    //Use Diff for the aux PID
    _testMotor.configSelectedFeedbackSensor(	FeedbackDevice.SensorDifference, 
                                              Constants.PID_TURN, 
                                              Constants.kTimeoutMs);

    //Don't scale the "turning" value
    _testMotor.configSelectedFeedbackCoefficient( 1, 
                                                    Constants.PID_TURN, 
                                                    Constants.kTimeoutMs);

    //Set status frame periods to ensure we don't have stale data
    _testMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
		_testMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
		_testMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);		
    //_otherMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);

    //Config motor deadband value
    _testMotor.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

    //Configure PID variables for primary PID (Velocity or Forward)
    _testMotor.config_kP(Constants.kSlot_Velocit, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
		_testMotor.config_kI(Constants.kSlot_Velocit, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
		_testMotor.config_kD(Constants.kSlot_Velocit, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
		_testMotor.config_kF(Constants.kSlot_Velocit, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
		_testMotor.config_IntegralZone(Constants.kSlot_Velocit, Constants.kGains_Velocit.kIzone, Constants.kTimeoutMs);
		_testMotor.configClosedLoopPeakOutput(Constants.kSlot_Velocit, Constants.kGains_Velocit.kPeakOutput, Constants.kTimeoutMs);
    _testMotor.configAllowableClosedloopError(Constants.kSlot_Velocit, 0, Constants.kTimeoutMs);

    //Configure PID variables for aux PID (Turning)
    _testMotor.config_kP(Constants.kSlot_Turning, Constants.kGains_Turning.kP, Constants.kTimeoutMs);
		_testMotor.config_kI(Constants.kSlot_Turning, Constants.kGains_Turning.kI, Constants.kTimeoutMs);
		_testMotor.config_kD(Constants.kSlot_Turning, Constants.kGains_Turning.kD, Constants.kTimeoutMs);
		_testMotor.config_kF(Constants.kSlot_Turning, Constants.kGains_Turning.kF, Constants.kTimeoutMs);
		_testMotor.config_IntegralZone(Constants.kSlot_Turning, Constants.kGains_Turning.kIzone, Constants.kTimeoutMs);
		_testMotor.configClosedLoopPeakOutput(Constants.kSlot_Turning, Constants.kGains_Turning.kPeakOutput, Constants.kTimeoutMs);
    _testMotor.configAllowableClosedloopError(Constants.kSlot_Turning, 0, Constants.kTimeoutMs);

    //Config update time for PID loops
    int closedLoopTimeMs = 1;
		_testMotor.configClosedLoopPeriod(0, closedLoopTimeMs, Constants.kTimeoutMs);
    _testMotor.configClosedLoopPeriod(1, closedLoopTimeMs, Constants.kTimeoutMs);

    //Use if using aux sensor
    _testMotor.configAuxPIDPolarity(false, Constants.kTimeoutMs);

    _testMotor.configClosedloopRamp(0.03, Constants.kTimeoutMs);

	}
	
	public void arcadeDrive(double speed, double turn) {

    speed = Robot.applyDeadzone(speed);
    turn = Robot.applyDeadzone(turn);

		if(_driveMode == 0) {

			//Do nothing

		} else if(_driveMode == 1) {

			_testMotor.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, turn);

		} else if(_driveMode == 2) {



		} else if(_driveMode == 3) {



		} else if(_driveMode == 11) {

      double targetSpeed = (Constants.driveTrainMaxUnitsPer100ms * Robot.m_driveTrainSensitivity) * speed;
      double targetTurn = (Constants.driveTrainMaxUnitsPer100ms * Robot.m_driveTrainSensitivity / 2) * turn;

      _lastSpeed = targetSpeed;
      _lastTurn = targetTurn;

      _testMotor.set(ControlMode.Velocity, targetSpeed, DemandType.AuxPID, targetTurn);

		} else if(_driveMode == 12) {



		} else if(_driveMode == 13) {



		} else {



    }

    

  }
	
	public void setSensitivity(double sen) {
		if(sen > 1 || sen < -1) {
			_sensitivity = 1;
		} else {
			_sensitivity = Math.abs(sen);
		}
	}

	public boolean setMode(int mode) {
    if(mode == _driveMode){
      return true;
    }
    _driveMode = mode;
    if(mode == 11) {
      _testMotor.selectProfileSlot(Constants.kSlot_Velocit, Constants.PID_PRIMARY);
      _testMotor.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);
    }
		return true;
	}

	public int getMode() {
		return _driveMode;
	}
	
	public double getSensitivity() {
		return _sensitivity;
  }

  public void zeroSensors() {
    _testMotor.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
  }

  public int getSensorPosition() {
    return _testMotor.getSelectedSensorPosition();
  }

  public int getSensorVelocity() {
    return _testMotor.getSelectedSensorVelocity();
  }

  public double getMotorOutputPercent() {
    return _testMotor.getMotorOutputPercent();
  }

  public double getLastSpeed() {
    return _lastSpeed;
  }
  
  public double getLastTurn() {
    return _lastTurn;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Sensor Position", this::getSensorPosition, null);
    builder.addDoubleProperty("Sensor Velocity", this::getSensorVelocity, null);
    builder.addDoubleProperty("Motor Output", this::getMotorOutputPercent, null);
    
    builder.addDoubleProperty("Joystick Output Y", this::getLastSpeed, null);
    builder.addDoubleProperty("Joystick Output X", this::getLastTurn, null);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new TalonSRXCommand());
  }
}
