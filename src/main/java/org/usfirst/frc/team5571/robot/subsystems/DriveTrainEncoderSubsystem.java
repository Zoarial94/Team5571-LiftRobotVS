package org.usfirst.frc.team5571.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import org.usfirst.frc.team5571.robot.*;
import org.usfirst.frc.team5571.robot.commands.DriveTrain.DriveTrainDriveEncoders;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

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

	double    _sensitivity;
	TalonSRX  _rightMotor, _leftMotor;
	double _lastSpeed, _lastTurn;
	
	public DriveTrainEncoderSubsystem() {
		
		_sensitivity = 0.6;

    //Get CAN motor
		_rightMotor = new TalonSRX(RobotMap.DRIVETRAIN_RIGHTTALON);
		_leftMotor = new TalonSRX(RobotMap.DRIVETRAIN_LEFTTALON);
		
    //Output to zero for safety
		_rightMotor.set(ControlMode.PercentOutput, 0);
		_leftMotor.set(ControlMode.PercentOutput, 0);
    //Reset to defaults
		_rightMotor.configFactoryDefault();
		_leftMotor.configFactoryDefault();
    //Brake/Coast when output is zero
		_rightMotor.setNeutralMode(NeutralMode.Coast); //NeutralMode.Coast or NeutralMode.Brake
		_leftMotor.setNeutralMode(NeutralMode.Coast); 
    //Invert output
		_rightMotor.setInverted(true);
		_leftMotor.setInverted(false);
    //Invert sensor output
		_rightMotor.setSensorPhase(false);
		_leftMotor.setSensorPhase(false);
    
	}
	
	public void arcadeDrive(double speed, double turn) {

    speed = Robot.applyDeadzone(speed);
    turn = Robot.applyDeadzone(turn);

		if(Robot.m_driveMode == 0) {

			//Do nothing

		} else if(Robot.m_driveMode == 1) {

			_rightMotor.set(ControlMode.PercentOutput, -speed, DemandType.ArbitraryFeedForward, -turn);
			_leftMotor.set(ControlMode.PercentOutput, -speed, DemandType.ArbitraryFeedForward, turn);

		} else if(Robot.m_driveMode == 2) {



		} else if(Robot.m_driveMode == 3) {



		} else if(Robot.m_driveMode == 11) {

      double targetSpeed = (Constants.driveTrainMaxUnitsPer100ms * Robot.m_driveTrainSensitivity) * speed;
      double targetTurn = (Constants.driveTrainMaxUnitsPer100ms * Robot.m_driveTrainSensitivity) * turn * 0.5;

			_lastSpeed = targetSpeed;
      _lastTurn = targetTurn;

			_leftMotor.set(ControlMode.Velocity, -targetSpeed, DemandType.AuxPID, targetTurn);
			_rightMotor.set(ControlMode.Velocity, -targetSpeed, DemandType.AuxPID, targetTurn);

		} else if(Robot.m_driveMode == 12) {



		} else if(Robot.m_driveMode == 13) {



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
    if(mode == Robot.m_driveMode){
      return true;
		}
		
		if(mode == 1) {
			Robot.m_driveMode = mode;
		}
    if(mode == 11) {
      _rightMotor.selectProfileSlot(Constants.kSlot_Velocit, Constants.PID_PRIMARY);
			_rightMotor.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);

			_leftMotor.selectProfileSlot(Constants.kSlot_Velocit, Constants.PID_PRIMARY);
			_leftMotor.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);

			configForConstVelocity();

			Robot.m_driveMode = mode;
    }
		return true;
	}

	public int getMode() {
		return Robot.m_driveMode;
	}
	
	public double getSensitivity() {
		return _sensitivity;
	}

	private void configForConstVelocity() {
		//THERE ARE TWO PID LOOPS: PRIMARY AND AUXILARY
																							
		_rightMotor.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder, 			// Local Feedback Source
																							Constants.PID_PRIMARY,					// PID Slot for Source [0, 1]
																							Constants.kTimeoutMs); 

		_leftMotor.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder, 			// Local Feedback Source
																							Constants.PID_PRIMARY,					// PID Slot for Source [0, 1]
																							Constants.kTimeoutMs); 

		_rightMotor.configRemoteFeedbackFilter(		_leftMotor.getDeviceID(),					// Device ID of Source
																							RemoteSensorSource.TalonSRX_SelectedSensor,	// Remote Feedback Source
																							Constants.REMOTE_1,						// Source number [0, 1]
																							Constants.kTimeoutMs); // Configuration Timeout

		_leftMotor.configRemoteFeedbackFilter(		_rightMotor.getDeviceID(),					// Device ID of Source
																							RemoteSensorSource.TalonSRX_SelectedSensor,	// Remote Feedback Source
																							Constants.REMOTE_1,						// Source number [0, 1]
																							Constants.kTimeoutMs); // Configuration Timeout
    
    //There are two SensorTerms: Sum and Diff
    //Sum will equal Sum0 + Sum1
    //Diff will equal Diff0 - Diff1

    //Set local quad as Sum0
    //Will be used as "Forward Distance"
		_rightMotor.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs);
		
    _leftMotor.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs);
    
    //Set local quad as Diff0
    //Will be used as "Turning"
    //Diff1 should be a remote sensor and Diff will show the difference between the left and right forward movement
    _rightMotor.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor1, Constants.kTimeoutMs);
		_rightMotor.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs);

		_leftMotor.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor1, Constants.kTimeoutMs);
		_leftMotor.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs);

    //Use Sum for the primary PID
    _rightMotor.configSelectedFeedbackSensor(	FeedbackDevice.SensorSum, 
													                    Constants.PID_PRIMARY,
																							Constants.kTimeoutMs);
																							
		_leftMotor.configSelectedFeedbackSensor(	FeedbackDevice.SensorSum, 
													                    Constants.PID_PRIMARY,
                                              Constants.kTimeoutMs);

    //Coefficient should be 0.5 if using two encoders for Sum
    _rightMotor.configSelectedFeedbackCoefficient(	1, 						              // Coefficient
                                                  	Constants.PID_PRIMARY,		// PID Slot of Source 
																										Constants.kTimeoutMs);
																										
		_leftMotor.configSelectedFeedbackCoefficient(		1, 						              // Coefficient
                                                  	Constants.PID_PRIMARY,		// PID Slot of Source 
                                                  	Constants.kTimeoutMs);
    
    //Use Diff for the aux PID
    _rightMotor.configSelectedFeedbackSensor(	FeedbackDevice.SensorDifference, 
                                              Constants.PID_TURN, 
																							Constants.kTimeoutMs);
																							
		_leftMotor.configSelectedFeedbackSensor(	FeedbackDevice.SensorDifference, 
                                              Constants.PID_TURN, 
                                              Constants.kTimeoutMs);

    //
    _rightMotor.configSelectedFeedbackCoefficient( 	0.5, 
                                                    Constants.PID_TURN, 
																										Constants.kTimeoutMs);
																										
		_leftMotor.configSelectedFeedbackCoefficient( 	0.5, 
                                                    Constants.PID_TURN, 
                                                    Constants.kTimeoutMs);

    //Set status frame periods to ensure we don't have stale data
    _rightMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
		_rightMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
		_rightMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);		
		_leftMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);
		
		_leftMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
		_leftMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
		_leftMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);		
    _rightMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);

    //Config motor deadband value
		_rightMotor.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
		_leftMotor.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

		_leftMotor.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
		_leftMotor.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
		_rightMotor.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
		_rightMotor.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);

    //Configure PID variables for primary PID (Velocity or Forward)
    _rightMotor.config_kP(Constants.kSlot_Velocit, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
		_rightMotor.config_kI(Constants.kSlot_Velocit, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
		_rightMotor.config_kD(Constants.kSlot_Velocit, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
		_rightMotor.config_kF(Constants.kSlot_Velocit, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
		_rightMotor.config_IntegralZone(Constants.kSlot_Velocit, Constants.kGains_Velocit.kIzone, Constants.kTimeoutMs);
		_rightMotor.configClosedLoopPeakOutput(Constants.kSlot_Velocit, Constants.kGains_Velocit.kPeakOutput, Constants.kTimeoutMs);
    _rightMotor.configAllowableClosedloopError(Constants.kSlot_Velocit, 0, Constants.kTimeoutMs);

    //Configure PID variables for aux PID (Turning)
    _rightMotor.config_kP(Constants.kSlot_Turning, Constants.kGains_Turning.kP, Constants.kTimeoutMs);
		_rightMotor.config_kI(Constants.kSlot_Turning, Constants.kGains_Turning.kI, Constants.kTimeoutMs);
		_rightMotor.config_kD(Constants.kSlot_Turning, Constants.kGains_Turning.kD, Constants.kTimeoutMs);
		_rightMotor.config_kF(Constants.kSlot_Turning, Constants.kGains_Turning.kF, Constants.kTimeoutMs);
		_rightMotor.config_IntegralZone(Constants.kSlot_Turning, Constants.kGains_Turning.kIzone, Constants.kTimeoutMs);
		_rightMotor.configClosedLoopPeakOutput(Constants.kSlot_Turning, Constants.kGains_Turning.kPeakOutput, Constants.kTimeoutMs);
		_rightMotor.configAllowableClosedloopError(Constants.kSlot_Turning, 0, Constants.kTimeoutMs);
		

		//Configure PID variables for primary PID (Velocity or Forward)
		_leftMotor.config_kP(Constants.kSlot_Velocit, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
		_leftMotor.config_kI(Constants.kSlot_Velocit, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
		_leftMotor.config_kD(Constants.kSlot_Velocit, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
		_leftMotor.config_kF(Constants.kSlot_Velocit, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
		_leftMotor.config_IntegralZone(Constants.kSlot_Velocit, Constants.kGains_Velocit.kIzone, Constants.kTimeoutMs);
		_leftMotor.configClosedLoopPeakOutput(Constants.kSlot_Velocit, Constants.kGains_Velocit.kPeakOutput, Constants.kTimeoutMs);
    _leftMotor.configAllowableClosedloopError(Constants.kSlot_Velocit, 0, Constants.kTimeoutMs);

    //Configure PID variables for aux PID (Turning)
    _leftMotor.config_kP(Constants.kSlot_Turning, Constants.kGains_Turning.kP, Constants.kTimeoutMs);
		_leftMotor.config_kI(Constants.kSlot_Turning, Constants.kGains_Turning.kI, Constants.kTimeoutMs);
		_leftMotor.config_kD(Constants.kSlot_Turning, Constants.kGains_Turning.kD, Constants.kTimeoutMs);
		_leftMotor.config_kF(Constants.kSlot_Turning, Constants.kGains_Turning.kF, Constants.kTimeoutMs);
		_leftMotor.config_IntegralZone(Constants.kSlot_Turning, Constants.kGains_Turning.kIzone, Constants.kTimeoutMs);
		_leftMotor.configClosedLoopPeakOutput(Constants.kSlot_Turning, Constants.kGains_Turning.kPeakOutput, Constants.kTimeoutMs);
    _leftMotor.configAllowableClosedloopError(Constants.kSlot_Turning, 0, Constants.kTimeoutMs);

    //Config update time for PID loops
    int closedLoopTimeMs = 1;
		_rightMotor.configClosedLoopPeriod(0, closedLoopTimeMs, Constants.kTimeoutMs);
		_rightMotor.configClosedLoopPeriod(1, closedLoopTimeMs, Constants.kTimeoutMs);

		_leftMotor.configClosedLoopPeriod(0, closedLoopTimeMs, Constants.kTimeoutMs);
    _leftMotor.configClosedLoopPeriod(1, closedLoopTimeMs, Constants.kTimeoutMs);

    //Use if using aux sensor
		_rightMotor.configAuxPIDPolarity(false, Constants.kTimeoutMs);
		_leftMotor.configAuxPIDPolarity(false, Constants.kTimeoutMs);

		_rightMotor.configClosedloopRamp(0.03, Constants.kTimeoutMs);
		_leftMotor.configClosedloopRamp(0.03, Constants.kTimeoutMs);
	}

	public int getSensorPositionRight() {
    return _rightMotor.getSelectedSensorPosition();
  }

  public int getSensorVelocityRight() {
    return _rightMotor.getSelectedSensorVelocity();
  }

  public double getMotorOutputPercentRight() {
    return _rightMotor.getMotorOutputPercent();
	}
	
	public int getSensorPositionLeft() {
    return _leftMotor.getSelectedSensorPosition();
  }

  public int getSensorVelocityLeft() {
    return _leftMotor.getSelectedSensorVelocity();
  }

  public double getMotorOutputPercentLeft() {
    return _leftMotor.getMotorOutputPercent();
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
    builder.addDoubleProperty("Right - Sensor Position", this::getSensorPositionRight, null);
    builder.addDoubleProperty("Right - Sensor Velocity", this::getSensorVelocityRight, null);
		builder.addDoubleProperty("Right - Motor Output", this::getMotorOutputPercentRight, null);
		

    builder.addDoubleProperty("Left - Sensor Position", this::getSensorPositionLeft, null);
    builder.addDoubleProperty("Left - Sensor Velocity", this::getSensorVelocityLeft, null);
    builder.addDoubleProperty("Left - Motor Output", this::getMotorOutputPercentLeft, null);
    
    builder.addDoubleProperty("Joystick Output Y", this::getLastSpeed, null);
    builder.addDoubleProperty("Joystick Output X", this::getLastTurn, null);

  }

	public void initDefaultCommand() {
		setDefaultCommand(new DriveTrainDriveEncoders());
	}
}

