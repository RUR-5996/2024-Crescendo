package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Subsystems.swerve.SwerveDef.pidValues;

public class Constants {
    public static final class DriverConstants{
        public static final double CONTROLLER_DEBOUNCE_TIME = 0.2;
        public static final double DRIVE_GOVERNOR = 1;
        public static final double TURN_GOVERNOR = 1;
        public static final double PRECISION_RATIO = 0.7;
    }
    public static final class AutoConstants { //TODO move to SwerveConstants
        public static final double t_kP = 15;
        public static final double r_kP = 15;
        public static final HolonomicPathFollowerConfig autoConfig = new HolonomicPathFollowerConfig(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0), SwerveConstants.MAX_SPEED_METERSperSECOND, Math.hypot(SwerveConstants.WHEEL_BASE_WIDTH, SwerveConstants.TRACK_WIDTH), new ReplanningConfig());
        public static final double MAX_ACCELERATION = 0.5; //TODO determine
        public static final double MAX_VELOCITY = 4.9; //TODO determine
        public static final double MAX_ROT_ACCELERATION = 0.5; //TODO determine
        public static final double MAX_ROT_VELOCITY = 2*Math.PI*Math.hypot(SwerveConstants.WHEEL_BASE_WIDTH, SwerveConstants.TRACK_WIDTH)*SwerveConstants.MAX_SPEED_METERSperSECOND; //TODO determine
    }
    public static final class IntakeConstants {
        public static final int motorID = 20;
        public static final double motorSpeed = 0.95;
        public static final InvertedValue inverted = InvertedValue.CounterClockwise_Positive;
    }
    public static final class ShooterConstants {
        public static final double INITIALIZED_ANGLE = 0; //TODO measure and correct
        public static final boolean ROTATION_INVERTED = true; //clockwise positive, counterclockwise negative
        public static final InvertedValue LONG_INVERTED = InvertedValue.CounterClockwise_Positive; //out is positive, in is negative
        public static final boolean SHORT_INVERTED = true; //out is positive, in is negative
        public static final int ROTATION_ID = 6;
        public static final int LONG_ID = 11;
        public static final int SHORT_ID = 10;
        public static final double ROTATION_POSITION_FACTOR = 1.0/53.33*360; //TODO fancy math
        public static final double ROTATION_VELOCITY_FACTOR = 0.5; //TODO fancy math, might be same as above 550-11000rpm
        public static final float ROTATION_LIMIT_FWD = 360; //TODO measure cable length and all that
        public static final float ROTATION_LIMIT_REV = 120; //TODO measure cable length and all that
        public static final int ROTATION_CURRENT_LIMIT = 30; 
        public static final double ROTATION_KP = 0.03;
        public static final double ROTATION_KI = 0.0;
        public static final double ROTATION_KD = 0.0;
        public static final int LONG_CURRENT_LIMIT = 30;
        public static final int SHORT_CURRENT_LIMIT = 30;
        public static final double ERROR = 2; //TODO try less
        public static final double SHOOT_SPEED = 0.85; //TODO make more cases by positions
        public static final double FEED_SPEED = 0.3;

        public static final double ROTATION_OFFSET = 165;
    }

    public static final class ClimberConstants {
        public static final int leftMotorID = 30;
        public static final int rightMotorID = 31;
        public static final boolean leftMotorInverted = false;
        public static final boolean rightMotorInverted = true;

        public static final int leftMaxTicks = 100;
        public static final int rightMaxTicks = 100;
    }

    public class SwerveConstants {
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kIzone = 300;
        public static final double kS = 0;
        public static final double kV = 0;

        public static final double DRIVE_MOTOR_GEARING = 6.92;
        public static final double WHEEL_RADIUS_METERS = 0.10/2.0;
        public static final double FALCON_RPM = 6379.0;
        public static final double DRIVE_FACTOR = FALCON_RPM / 60.0 / DRIVE_MOTOR_GEARING * WHEEL_RADIUS_METERS; //mps

        public static final double WHEEL_BASE_WIDTH = 0.52;
        public static final double TRACK_WIDTH = 0.525;

        public static final double MAX_SPEED_TICKSper100MS = 21900;

        public static final double SECONDSper100MS = .1;
        public static final double TICKSperTALONFX_Rotation = 2048;
        public static final double DRIVE_MOTOR_TICKSperREVOLUTION = DRIVE_MOTOR_GEARING * TICKSperTALONFX_Rotation;
        public static final double METERSperWHEEL_REVOLUTION = 2 * Math.PI * WHEEL_RADIUS_METERS;
        public static final double METERSperROBOT_REVOLUTION = 2 * Math.PI
            * Math.hypot(TRACK_WIDTH, WHEEL_BASE_WIDTH);
        public static final double MAX_SPEED_METERSperSECOND = MAX_SPEED_TICKSper100MS / SECONDSper100MS
            / DRIVE_MOTOR_TICKSperREVOLUTION * METERSperWHEEL_REVOLUTION;
        public static final double MAX_SPEED_RADIANSperSECOND = MAX_SPEED_METERSperSECOND / METERSperROBOT_REVOLUTION
            * (2 * Math.PI);
        //public static final double MAX_SPEED_MPS = 4.9; //TODO get precise values
        public static final double P_ROTATION_CONTROLLER = 0.072;
        public static final double I_ROTATION_CONTROLLER = 0.0;
        public static final double D_ROTATION_CONTROLLER = 0.0;

        public static final double STEER_FEEDBACK_COEFFICIENT = 1.0 / 18.0 * 360.0;

        public static final pidValues FL_STEER_PID_VALUES = new pidValues(0.01, 0, 0);
        public static final pidValues FR_STEER_PID_VALUES = new pidValues(0.01, 0, 0);
        public static final pidValues RL_STEER_PID_VALUES = new pidValues(0.01, 0, 0);
        public static final pidValues RR_STEER_PID_VALUES = new pidValues(0.01, 0, 0);

        public static final pidValues FL_DRIVE_PID_VALUES = new pidValues(1, 0, 0);
        public static final pidValues FR_DRIVE_PID_VALUES = new pidValues(1, 0, 0);
        public static final pidValues RL_DRIVE_PID_VALUES = new pidValues(1, 0, 0);
        public static final pidValues RR_DRIVE_PID_VALUES = new pidValues(1, 0, 0);

        public static final boolean FL_STEER_INVERT_TYPE = false;
        public static final boolean FR_STEER_INVERT_TYPE = false;
        public static final boolean RL_STEER_INVERT_TYPE = false;
        public static final boolean RR_STEER_INVERT_TYPE = false;
        
        public static final InvertedValue FL_DRIVE_INVERT_TYPE = InvertedValue.Clockwise_Positive;
        public static final InvertedValue FR_DRIVE_INVERT_TYPE = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue RL_DRIVE_INVERT_TYPE = InvertedValue.Clockwise_Positive;
        public static final InvertedValue RR_DRIVE_INVERT_TYPE = InvertedValue.CounterClockwise_Positive;

        //TODO remove?
        public static final double FL_STEER_OFFSET = 118.7 - 5.76;
        public static final double FR_STEER_OFFSET = 111.1 - 1.14 + 15.8;
        public static final double RL_STEER_OFFSET = 20.05 + 1.65;
        public static final double RR_STEER_OFFSET = 124.2 - 3.17;

        public static final Translation2d FL_LOC = new Translation2d(SwerveConstants.WHEEL_BASE_WIDTH / 2, SwerveConstants.TRACK_WIDTH / 2);
        public static final Translation2d FR_LOC = new Translation2d(SwerveConstants.WHEEL_BASE_WIDTH / 2, -SwerveConstants.TRACK_WIDTH / 2);
        public static final Translation2d RL_LOC = new Translation2d(-SwerveConstants.WHEEL_BASE_WIDTH / 2, SwerveConstants.TRACK_WIDTH / 2);
        public static final Translation2d RR_LOC = new Translation2d(-SwerveConstants.WHEEL_BASE_WIDTH / 2, -SwerveConstants.TRACK_WIDTH / 2);

    }   
}
