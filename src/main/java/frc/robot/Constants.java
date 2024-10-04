// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.SwerveDef.pidValues;

public final class Constants {
  public static class SwerveConstants {
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kIzone = 300;
    public static final double kS = 0;
    public static final double kV = 0;

    public static final double DRIVE_MOTOR_GEARING = 5.355;
    public static final double STEER_MOTOR_GEARING = 21.43;
    public static final double STEER_MOTOR_COEFFICIENT = 1.0 / STEER_MOTOR_GEARING * 360.0;
    public static final double WHEEL_RADIUS_METERS = 0.10/2;
    public static final double FALCON_RPM = 6379.0*1.15;
    public static final double DRIVE_FACTOR = FALCON_RPM / (60.0 * DRIVE_MOTOR_GEARING) * 2 * Math.PI * WHEEL_RADIUS_METERS; //mps

    public static final double WHEEL_BASE_WIDTH = 0.517;
    public static final double TRACK_WIDTH = 0.516;

    //public static final double MAX_SPEED_TICKSper100MS = 21900;
    public static final double SECONDSper100MS = .1;
    public static final double TICKSperTALONFX_Rotation = 2048;
    public static final double DRIVE_MOTOR_TICKSperREVOLUTION = DRIVE_MOTOR_GEARING * TICKSperTALONFX_Rotation;
    public static final double METERSperWHEEL_REVOLUTION = 2 * Math.PI * WHEEL_RADIUS_METERS;
    public static final double METERSperROBOT_REVOLUTION = 2 * Math.PI
            * Math.hypot(TRACK_WIDTH, WHEEL_BASE_WIDTH);
    public static final double MAX_SPEED_METERSperSECOND = DRIVE_FACTOR;        
    //public static final double MAX_SPEED_METERSperSECOND = 6.34;
    public static final double MAX_SPEED_RADIANSperSECOND = MAX_SPEED_METERSperSECOND / METERSperROBOT_REVOLUTION
    * (2 * Math.PI);
    public static final double P_ROTATION_CONTROLLER = 0.055;
    public static final double I_ROTATION_CONTROLLER = 0.0;
    public static final double D_ROTATION_CONTROLLER = 0.0;

    public static final Translation2d FL_LOC = new Translation2d(SwerveConstants.WHEEL_BASE_WIDTH / 2, SwerveConstants.TRACK_WIDTH / 2);
    public static final Translation2d FR_LOC = new Translation2d(SwerveConstants.WHEEL_BASE_WIDTH / 2, -SwerveConstants.TRACK_WIDTH / 2);
    public static final Translation2d RL_LOC = new Translation2d(-SwerveConstants.WHEEL_BASE_WIDTH / 2, SwerveConstants.TRACK_WIDTH / 2);
    public static final Translation2d RR_LOC = new Translation2d(-SwerveConstants.WHEEL_BASE_WIDTH / 2, -SwerveConstants.TRACK_WIDTH / 2);

    public static final InvertedValue FL_DRIVE_INVERT_TYPE = InvertedValue.Clockwise_Positive;
    public static final InvertedValue FR_DRIVE_INVERT_TYPE = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue RL_DRIVE_INVERT_TYPE = InvertedValue.Clockwise_Positive;
    public static final InvertedValue RR_DRIVE_INVERT_TYPE = InvertedValue.CounterClockwise_Positive;

    public static final boolean FL_STEER_INVERT_TYPE = true;
    public static final boolean FR_STEER_INVERT_TYPE = true;
    public static final boolean RL_STEER_INVERT_TYPE = true;
    public static final boolean RR_STEER_INVERT_TYPE = true;

    public static final double FL_STEER_OFFSET = 0;
    public static final double FR_STEER_OFFSET = 0;
    public static final double RL_STEER_OFFSET = 0;
    public static final double RR_STEER_OFFSET = 0;

    public static final pidValues FL_STEER_PID_VALUES = new pidValues(0.015, 0, 0);
    public static final pidValues FR_STEER_PID_VALUES = new pidValues(0.015, 0, 0);
    public static final pidValues RL_STEER_PID_VALUES = new pidValues(0.015, 0, 0);
    public static final pidValues RR_STEER_PID_VALUES = new pidValues(0.015, 0, 0);

    public static final pidValues FL_DRIVE_PID_VALUES = new pidValues(1, 0, 0);
    public static final pidValues FR_DRIVE_PID_VALUES = new pidValues(1, 0, 0);
    public static final pidValues RL_DRIVE_PID_VALUES = new pidValues(1, 0, 0);
    public static final pidValues RR_DRIVE_PID_VALUES = new pidValues(1, 0, 0);
  }
  
  public static class DriverConstants {
    public static final double CONTROLLER_DEBOUNCE_TIME = 0.2;
    public static final double DRIVE_GOVERNOR = 1;
    public static final double TURN_GOVERNOR = 1;
    public static final double PRECISION_RATIO = 0.2;
  }
  
  public static final class AutoConstants { //TODO move to SwerveConstants
    public static final double t_kP = 15;
    public static final double r_kP = 15;
    public static final HolonomicPathFollowerConfig autoConfig = new HolonomicPathFollowerConfig(new PIDConstants(8.0, 0.0, 0.0), new PIDConstants(0, 0.0, 0.0), SwerveConstants.MAX_SPEED_METERSperSECOND, Math.hypot(SwerveConstants.WHEEL_BASE_WIDTH, SwerveConstants.TRACK_WIDTH), new ReplanningConfig());
    //public static final double MAX_ACCELERATION = 0.5; //TODO determine
    //public static final double MAX_VELOCITY = 4.9; //TODO determine
    //public static final double MAX_ROT_ACCELERATION = 0.5; //TODO determine
   // public static final double MAX_ROT_VELOCITY = 2*Math.PI*Math.hypot(SwerveConstants.WHEEL_BASE_WIDTH, SwerveConstants.TRACK_WIDTH)*SwerveConstants.MAX_SPEED_METERSperSECOND; //TODO determine
  }
}
