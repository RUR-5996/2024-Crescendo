package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.Subsystems.swerve.SwerveConstants;

public class Constants {
    public static final class DriverConstants{
        public static final double DRIVE_GOVERNOR = 1; //TODO 0-1 for training purpose
        public static final boolean FIELD_RELATIVE = true;
        public static final boolean ACCELERATED_INPUTS = false;
    }
    public static final class AutoConstants {
        public static final double t_kP = 15;
        public static final double r_kP = 15;
        public static final HolonomicPathFollowerConfig autoConfig = new HolonomicPathFollowerConfig(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0), SwerveConstants.MAX_SPEED_METERSperSECOND, SwerveConstants.DRIVE_BASE_RADIUS, new ReplanningConfig());
        public static final double MAX_ACCELERATION = 0; //TODO determine
        public static final double MAX_VELOCITY = 0; //TODO determine
        public static final double MAX_ROT_ACCELERATION = 0; //TODO determine
        public static final double MAX_ROT_VELOCITY = 0; //TODO determine
    }
    public static final class IntakeConstants {
        public static final boolean INVERTED_MOTOR = false; //up is positive, down is negative
        public static final double INTAKE_DISTANCE = 100; //TODO make correct estimate
        public static final double INTAKE_SPEED = 0.7; //TODO adjust
        public static final double MAX_SPEED = 0.9;
    }
    public static final class ShooterConstants {
        public static final double INITIALIZED_ANGLE = 0; //TODO measure and correct
        public static final boolean ROTATION_INVERTED = false; //clockwise positive, counterclockwise negative
        public static final boolean LONG_INVERTED = false; //out is positive, in is negative
        public static final boolean SHORT_INVERTED = false; //out is positive, in is negative
        public static final int ROTATION_ID = 6;
        public static final int LONG_ID = 7;
        public static final int SHORT_ID = 8;
        public static final double ROTATION_POSITION_FACTOR = 0; //TODO fancy math
        public static final double ROTATION_VELOCITY_FACTOR = 0; //TODO fancy math, might be same as above
        public static final float ROTATION_LIMIT_FWD = 360; //TODO measure cable length and all that
        public static final float ROTATION_LIMIT_REV = 360; //TODO measure cable length and all that
        public static final int ROTATION_CURRENT_LIMIT = 10; 
        public static final double ROTATION_KP = 0.05;
        public static final double ROTATION_KI = 0.0;
        public static final double ROTATION_KD = 0.0;
        public static final int LONG_CURRENT_LIMIT = 30;
        public static final int SHORT_CURRENT_LIMIT = 30;
        public static final double ERROR = 2; //TODO try less
        public static final double SHOOT_SPEED = 0.85; //TODO make more cases by positions
        public static final double FEED_SPEED = 0.3;
    }
}
