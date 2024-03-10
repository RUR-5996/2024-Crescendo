package frc.robot.Subsystems.swerve;

import com.ctre.phoenix6.signals.InvertedValue;

public class SwerveConstants {

    //chassis characteristics
    public static final double TRACK_WIDE_METERS = 0.5; //TODO zmerit v CADu
    public static final double WHEEL_BASE_METERS = 0.5; //TODO zmerit v CADu
    public static final double DRIVE_BASE_RADIUS = Math.sqrt(TRACK_WIDE_METERS * TRACK_WIDE_METERS + WHEEL_BASE_METERS * WHEEL_BASE_METERS); //should be good enough
    public static final double P_ROTATION_CONTROLLER = 6.0;
    public static final double I_ROTATION_CONTROLLER = 0.0;
    public static final double D_ROTATION_CONTROLLER = 0.0;
    public static final double MAX_SPEED_METERSperSECOND = 4.9; //TODO check calc
    public static final double MAX_SPEED_RADIANSperSECOND = 2*Math.PI*DRIVE_BASE_RADIUS*MAX_SPEED_METERSperSECOND; //TODO check calc
    public static final boolean OPTIMIZESTEERING = true;

    //FL module
    public static int FLDriveID = 1;
    public static int FLSteerID = 1;
    public static Gains FLDriveGains = new Gains(1, 0.0, 0.0, 1023.0/20660.0);
    public static Gains FLSteerGains = new Gains(0.01, 0.0, 0.0);
    public static InvertedValue FLDriveInverted = InvertedValue.Clockwise_Positive;
    public static boolean FLSteerInverted = false;

    //FR module
    public static int FRDriveID = 2;
    public static int FRSteerID = 2;
    public static Gains FRDriveGains = new Gains(1, 0.0, 0.0, 1023.0/20660.0);
    public static Gains FRSteerGains = new Gains(0.01, 0.0, 0.0);
    public static InvertedValue FRDriveInverted = InvertedValue.Clockwise_Positive;
    public static boolean FRSteerInverted = false;

    //RL module
    public static int RLDriveID = 3;
    public static int RLSteerID = 3;
    public static Gains RLDriveGains = new Gains(1, 0.0, 0.0, 1023.0/20660.0);
    public static Gains RLSteerGains = new Gains(0.01, 0.0, 0.0);
    public static InvertedValue RLDriveInverted = InvertedValue.Clockwise_Positive;
    public static boolean RLSteerInverted = false;

    //FL module
    public static int RRDriveID = 4;
    public static int RRSteerID = 4;
    public static Gains RRDriveGains = new Gains(1, 0.0, 0.0, 1023.0/20660.0);
    public static Gains RRSteerGains = new Gains(0.01, 0.0, 0.0);
    public static InvertedValue RRDriveInverted = InvertedValue.Clockwise_Positive;
    public static boolean RRSteerInverted = false;
    
    public static final double DRIVE_MOTOR_GEARING = 6.92; //TODO should be correct based on vendor data
    public static final double kSteerConversionFactor = 1.0 / 18.0; //TODO check if correct - 18 rotations of motor = 1 module rotation
    public static final double WHEEL_RADIUS_METERS = 2*2.54/100; //should be in meters

    //FeedForward controller values, stolen baseline
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kP = 0.1;

    //default CTRE contstants
    public static final int DEFAULT_PID_SLOT_ID = 0;
    public static final int DEFAULT_CLOSED_LOOP_ERROR = 1; //degrees?
    public static final int DEFAULT_TIMEOUT = 30; //ms

    //Method for packaging PID values
    public static class Gains {
        public final double kP;
        public final double kI;
        public final double kD;
        public final double kF;
        public final double kIzone;
        public final double kPeakOutput;

        public Gains(double _kP, double _kI, double _kD) {
            kP = _kP;
            kI = _kI;
            kD = _kD;
            kF = 0;
            kIzone = 0;
            kPeakOutput = 1;
        }

        public Gains(double _kP, double _kI, double _kD, double _kF) {
            kP = _kP;
            kI = _kI;
            kD = _kD;
            kF = _kF;
            kIzone = 300;
            kPeakOutput = 1;
        }
    }
}
