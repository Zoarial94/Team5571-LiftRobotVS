/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5571.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team5571.robot.subsystems.*;
import org.usfirst.frc.team5571.robot.commands.*;
import org.usfirst.frc.team5571.robot.commands.DriveTrain.*;
import org.usfirst.frc.team5571.robot.commands.Claw.*;
import org.usfirst.frc.team5571.robot.commands.Elevator.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	


	//Subsystems
	public static DriveTrainSubsystem m_DriveTrainSub;
	public static DriveTrainEncoderSubsystem m_DriveTrainEncoderSub;
	public static TalonSRXTesting m_TalonSRX;

	public static ElevatorSubsystem m_ElevatorSub;
	public static ClawSubsystem m_ClawSub;
	
	public static OI m_oi;

	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser;
	public static SendableChooser<Integer> m_TalonMode;

	//Robot Variables
	public static double m_driveTrainSensitivity = Constants.driveTrainNormalSpeed;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		//SmartDashboard.putData(Scheduler.getInstance());
		
		m_chooser = new SendableChooser<>();
		m_TalonMode = new SendableChooser<>();

		m_oi = new OI();
		
		//m_DriveTrainEncoderSub  = new DriveTrainEncoderSubsystem();
		//m_DriveTrainSub         = new DriveTrainSubsystem();
		m_ElevatorSub           = new ElevatorSubsystem();
		m_ClawSub               = new ClawSubsystem();
		m_TalonSRX              = new TalonSRXTesting();
		//m_MechanumDriveTrainSub = new MechanumDriveTrain();

		SmartDashboard.putData(m_ElevatorSub);
		SmartDashboard.putData(m_ClawSub);
		SmartDashboard.putData(m_TalonSRX);

		SmartDashboard.putData(new resetSensors());

		m_TalonMode.addOption("1", 1);
		m_TalonMode.addOption("11", 11);
		m_TalonMode.setDefaultOption("11", 11);
		
		m_chooser.addOption("Raise Elevator", new ElevatorRaise());
		m_chooser.addOption("Lower Elevator", new ElevatorLower());
		SmartDashboard.putData("Auto Mode", m_chooser);
		SmartDashboard.putData("Talon Mode", m_TalonMode);
		
		m_oi.LB.whileHeld(new ElevatorLower());
		m_oi.RB.whileHeld(new ElevatorRaise());
		m_oi.X.whenPressed(new SetMaximumOverdrive());
		m_oi.X.whenReleased(new SetNormalSpeed());
		/*
		m_oi.LT.whileHeld(new ClawDriveOut());
		m_oi.RT.whileHeld(new ClawDriveIn());  
		*/
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_chooser.getSelected();

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}

	public static double applyDeadzone(double position) {
		if(position == 0.0)
			return 0.0;

		boolean positive = position > 0 ? true : false;

		double abs = Math.abs(position);

		if(abs >= 1) {
			return positive ? 1 : -1;
		} else if(abs < Constants.Deadzone) {
			return 0.0;
		} else {
			return positive ? (abs - Constants.Deadzone) / (1.0 - Constants.Deadzone) : -1.0 * ((abs - Constants.Deadzone) / (1.0 - Constants.Deadzone));
		}
	}

}
