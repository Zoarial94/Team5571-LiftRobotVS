package org.usfirst.frc.team5571.robot;
import org.usfirst.frc.team5571.robot.Gains;

public class Constants {

	/**
	 * Drive Train Constants
	 */
	public final static double driveTrainNormalSpeed = 0.4;
	public final static double driveTrainMaxSpeed = 1.0;
	
	public final static double driveTrainMaxUnitsPer100ms = 700.0;

	/**
	 * Controller Constants
	 */
	public final static double Deadzone = 0.07;

    /**
	 * How many sensor units per rotation.
	 * Using CTRE Magnetic Encoder.
	 * @link https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
	 */
    public final static int kSensorUnitsPerRotation = 1102;

    /**
	 * Set to zero to skip waiting for confirmation.
	 * Set to nonzero to wait and report to DS if action fails.
	 */
    public final static int kTimeoutMs = 30;

	public final static double kNeutralDeadband = 0.076;
	
	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 6800 represents Velocity units at 100% output
     * Not all set of Gains are used in this project and may be removed as desired.
     * 
	 * 	                                    			  kP   kI   kD   kF               Iz    PeakOut */
	public final static Gains kGains_Distanc = new Gains( 0.1, 0.0,  0.0, 0.0,            100,  1.00 );
	public final static Gains kGains_Turning = new Gains( 0.45, 0.02,  16.0, 1023.0/driveTrainMaxUnitsPer100ms,   80,  0.5 );
	public final static Gains kGains_Velocit = new Gains( 0.45, 0.02,  16.0, 1023.0/driveTrainMaxUnitsPer100ms,   80,  1.00 );
	public final static Gains kGains_MotProf = new Gains( 1.0, 0.0,  0.0, 1023.0/driveTrainMaxUnitsPer100ms,   400,  1.00 );

    /** ---- Flat constants, you should not need to change these ---- */
	/* We allow either a 0 or 1 when selecting an ordinal for remote devices [You can have up to 2 devices assigned remotely to a talon/victor] */
	public final static int REMOTE_0 = 0;
	public final static int REMOTE_1 = 1;
	/* We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1 is auxiliary */
	public final static int PID_PRIMARY = 0;
	public final static int PID_TURN = 1;
	/* Firmware currently supports slots [0, 3] and can be used for either PID Set */
	public final static int SLOT_0 = 0;
	public final static int SLOT_1 = 1;
	public final static int SLOT_2 = 2;
	public final static int SLOT_3 = 3;
	/* ---- Named slots, used to clarify code ---- */
	public final static int kSlot_Distanc = SLOT_0;
	public final static int kSlot_Turning = SLOT_1;
	public final static int kSlot_Velocit = SLOT_2;
    public final static int kSlot_MotProf = SLOT_3;

}