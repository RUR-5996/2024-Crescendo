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
}
