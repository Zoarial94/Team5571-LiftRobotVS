package org.usfirst.frc.team5571.robot.subsystems;

import java.util.Date;

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
	double _lastSpeed, _lastTurn, _lastDistance;
	boolean _turnAroundCenter;
	QueueCommand _currentCommand;
	long _time;
	Date _date;
	
	/**
	 * 0 - Stopped
	 * 1 - Moving
	 * 2 - Coasting/Braking
	 */
	int _drivingStatus;
	
	public DriveTrainEncoderSubsystem() {

		_date = new Date();
		
		_sensitivity = 0.6;

    //Get CAN motor
		_rightMotor = new TalonSRX(RobotMap.DRIVETRAIN_RIGHTTALON);
		_leftMotor = new TalonSRX(RobotMap.DRIVETRAIN_LEFTTALON);
		
		//Output to zero for safety
		stopMotors();
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

		_turnAroundCenter = false;
    
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
			
			if(_turnAroundCenter) {
				targetSpeed = 0;
			}
			
			double leftMotorSpeed = -targetSpeed + targetTurn;
			double rightMotorSpeed = -targetSpeed - targetTurn;

			//Prevents jitter if going from high throttle to low throttle
			double newDistance = Math.sqrt(Math.pow(targetSpeed, 2) + Math.pow(targetTurn * 2, 2));

			if(newDistance > 0) {
				_drivingStatus = 1;
			} 
			
			if(_drivingStatus == 0) {
				_leftMotor.set(ControlMode.Velocity, 0);
				_rightMotor.set(ControlMode.Velocity, 0);

			} else if((newDistance == 0.0) || _lastDistance - newDistance > 150) {
				
				if((getSensorVelocityLeft() == 0 && getSensorVelocityRight() == 0)) {

					_drivingStatus = 0;
					_leftMotor.set(ControlMode.Velocity, 0);
					_rightMotor.set(ControlMode.Velocity, 0);

				} else {

					_drivingStatus = 2;
					_leftMotor.set(ControlMode.PercentOutput, 0);
					_rightMotor.set(ControlMode.PercentOutput, 0);

				}

			} else {

				_drivingStatus = 1;
				_leftMotor.set(ControlMode.Velocity, leftMotorSpeed);
				_rightMotor.set(ControlMode.Velocity, rightMotorSpeed);

			}

			_lastDistance = newDistance;
			_lastSpeed = targetSpeed;
			_lastTurn = targetTurn;

		} else if(Robot.m_driveMode == 12) {

			if(_currentCommand == null && !Robot.m_queue.isEmpty()) {

				_currentCommand = Robot.m_queue.firstElement();
				zeroSensors();
				_drivingStatus = 1;

			} else if(_currentCommand == null && Robot.m_queue.isEmpty()) {

				_drivingStatus = 0;
				stopMotors();
				return;

			} 

			int rightValue = _currentCommand.getRightTargetValue();
			int leftValue = _currentCommand.getLeftTargetValue();

			_leftMotor.set(ControlMode.Position, leftValue);
			_rightMotor.set(ControlMode.Position, rightValue);

			if(Math.abs(getSensorPositionLeft() - leftValue) <= 10 && Math.abs(getSensorPositionRight() - rightValue) <= 10) {
				if (_time == -1) {
					_time = System.currentTimeMillis();
				}
			} else {
				_time = -1;
			}

			System.out.println("Left:  " + getSensorPositionLeft());
			System.out.println("Right: " + getSensorPositionRight());

			long time = System.currentTimeMillis();

			System.out.println("Start Time:   " + _time);
			System.out.println("Current Time: " + time);
			System.out.println("Time Difference: " + String.valueOf(time - _time));


			if(_time != -1 && time - _time >= 1500) {
				Robot.m_queue.remove(0);
				_currentCommand = null;
			} else {

				

			}

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
		} else if(mode == 11) {
      _rightMotor.selectProfileSlot(Constants.kSlot_Velocit, Constants.PID_PRIMARY);
			//_rightMotor.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);

			_leftMotor.selectProfileSlot(Constants.kSlot_Velocit, Constants.PID_PRIMARY);
			//_leftMotor.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);

			configForConstVelocity();

			Robot.m_driveMode = mode;
    } else if(mode == 12) {

			zeroSensors();
			_time = -1;

			
			configForPosition();

			_rightMotor.selectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
			_leftMotor.selectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);


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

	private void configForPosition() {

		configForConstVelocity();

		_leftMotor.configPeakOutputForward(+1.0 * Constants.driveTrainNormalSpeed, Constants.kTimeoutMs);
		_leftMotor.configPeakOutputReverse(-1.0 * Constants.driveTrainNormalSpeed, Constants.kTimeoutMs);
		_rightMotor.configPeakOutputForward(+1.0 * Constants.driveTrainNormalSpeed, Constants.kTimeoutMs);
		_rightMotor.configPeakOutputReverse(-1.0 * Constants.driveTrainNormalSpeed, Constants.kTimeoutMs);

		_rightMotor.config_kP(Constants.kSlot_Distanc , Constants.kGains_Distanc.kP, Constants.kTimeoutMs);
		_rightMotor.config_kI(Constants.kSlot_Distanc, Constants.kGains_Distanc.kI, Constants.kTimeoutMs);
		_rightMotor.config_kD(Constants.kSlot_Distanc, Constants.kGains_Distanc.kD, Constants.kTimeoutMs);
		_rightMotor.config_kF(Constants.kSlot_Distanc, Constants.kGains_Distanc.kF, Constants.kTimeoutMs);
		_rightMotor.config_IntegralZone(Constants.kSlot_Distanc, Constants.kGains_Distanc.kIzone, Constants.kTimeoutMs);
		_rightMotor.configClosedLoopPeakOutput(Constants.kSlot_Distanc, Constants.kGains_Distanc.kPeakOutput, Constants.kTimeoutMs);
		_rightMotor.configAllowableClosedloopError(Constants.kSlot_Distanc, 4, Constants.kTimeoutMs);
		
		_leftMotor.config_kP(Constants.kSlot_Distanc, Constants.kGains_Distanc.kP, Constants.kTimeoutMs);
		_leftMotor.config_kI(Constants.kSlot_Distanc, Constants.kGains_Distanc.kI, Constants.kTimeoutMs);
		_leftMotor.config_kD(Constants.kSlot_Distanc, Constants.kGains_Distanc.kD, Constants.kTimeoutMs);
		_leftMotor.config_kF(Constants.kSlot_Distanc, Constants.kGains_Distanc.kF, Constants.kTimeoutMs);
		_leftMotor.config_IntegralZone(Constants.kSlot_Distanc, Constants.kGains_Distanc.kIzone, Constants.kTimeoutMs);
		_leftMotor.configClosedLoopPeakOutput(Constants.kSlot_Distanc, Constants.kGains_Distanc.kPeakOutput, Constants.kTimeoutMs);
    _leftMotor.configAllowableClosedloopError(Constants.kSlot_Distanc, 4, Constants.kTimeoutMs);

	}

	private void configForConstVelocity() {
		//THERE ARE TWO PID LOOPS: PRIMARY AND AUXILARY
																							
		_rightMotor.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder, 			// Local Feedback Source
																							Constants.PID_PRIMARY,					// PID Slot for Source [0, 1]
																							Constants.kTimeoutMs); 

		_leftMotor.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder, 			// Local Feedback Source
																							Constants.PID_PRIMARY,					// PID Slot for Source [0, 1]
																							Constants.kTimeoutMs); 
    //There are two SensorTerms: Sum and Diff
    //Sum will equal Sum0 + Sum1
    //Diff will equal Diff0 - Diff1

    //Set local quad as Sum0
    //Will be used as "Forward Distance"
		_rightMotor.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs);
		
    _leftMotor.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs);

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

    //Set status frame periods to ensure we don't have stale data
    //_rightMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
		_rightMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
		_rightMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);		
		//_leftMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);
		
		//_leftMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
		_leftMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
		_leftMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);		
    //_rightMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);

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

		//Configure PID variables for primary PID (Velocity or Forward)
		_leftMotor.config_kP(Constants.kSlot_Velocit, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
		_leftMotor.config_kI(Constants.kSlot_Velocit, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
		_leftMotor.config_kD(Constants.kSlot_Velocit, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
		_leftMotor.config_kF(Constants.kSlot_Velocit, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
		_leftMotor.config_IntegralZone(Constants.kSlot_Velocit, Constants.kGains_Velocit.kIzone, Constants.kTimeoutMs);
		_leftMotor.configClosedLoopPeakOutput(Constants.kSlot_Velocit, Constants.kGains_Velocit.kPeakOutput, Constants.kTimeoutMs);
    _leftMotor.configAllowableClosedloopError(Constants.kSlot_Velocit, 0, Constants.kTimeoutMs);

    //Config update time for PID loops
    int closedLoopTimeMs = 1;
		_rightMotor.configClosedLoopPeriod(0, closedLoopTimeMs, Constants.kTimeoutMs);
		_rightMotor.configClosedLoopPeriod(1, closedLoopTimeMs, Constants.kTimeoutMs);

		_leftMotor.configClosedLoopPeriod(0, closedLoopTimeMs, Constants.kTimeoutMs);
    _leftMotor.configClosedLoopPeriod(1, closedLoopTimeMs, Constants.kTimeoutMs);

		_rightMotor.configClosedloopRamp(0.03, Constants.kTimeoutMs);
		_leftMotor.configClosedloopRamp(0.03, Constants.kTimeoutMs);
	}

	public void zeroSensors() {
		_leftMotor.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		_rightMotor.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
	}
	
	public void setTurnAroundCenter(boolean b) {
		_turnAroundCenter = b;
	}

	public void setMotorCoast(boolean b) {
		if(b) {
			_rightMotor.setNeutralMode(NeutralMode.Coast);
			_leftMotor.setNeutralMode(NeutralMode.Coast); 
		} else {
			_rightMotor.setNeutralMode(NeutralMode.Brake);
			_leftMotor.setNeutralMode(NeutralMode.Brake); 
		}
	}

	public void stopMotors() {

		_rightMotor.set(ControlMode.PercentOutput, 0);
		_leftMotor.set(ControlMode.PercentOutput, 0);

	}

	/**
	 * 
	 * Functions for getting information
	 * 
	 */

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

