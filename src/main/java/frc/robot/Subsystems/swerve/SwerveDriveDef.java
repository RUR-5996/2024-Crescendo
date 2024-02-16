package frc.robot.Subsystems.swerve;

import java.util.List;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Subsystems.swerve.SwerveModule.DriveMotor;
import frc.robot.Subsystems.swerve.SwerveModule.SteerMotor;

public class SwerveDriveDef {
    public NeutralModeValue NeutralMode = NeutralModeValue.Brake;

    public SwerveModule FLModule = new SwerveModule(
        new SwerveModule.DriveMotor(SwerveConstants.FLDriveID, SwerveConstants.FLDriveGains, SwerveConstants.FLDriveInverted),
        new SwerveModule.SteerMotor(SwerveConstants.FLSteerID, SwerveConstants.FLSteerGains, SwerveConstants.FLSteerInverted),
        new Translation2d(SwerveConstants.WHEEL_BASE_METERS / 2, SwerveConstants.TRACK_WIDE_METERS / 2));
        
    public SwerveModule FRModule = new SwerveModule(
        new SwerveModule.DriveMotor(SwerveConstants.FRDriveID, SwerveConstants.FRDriveGains, SwerveConstants.FRDriveInverted),
        new SwerveModule.SteerMotor(SwerveConstants.FRSteerID, SwerveConstants.FRSteerGains, SwerveConstants.FRSteerInverted),
        new Translation2d(SwerveConstants.WHEEL_BASE_METERS / 2, -SwerveConstants.TRACK_WIDE_METERS / 2));

    public SwerveModule RLModule = new SwerveModule(
        new SwerveModule.DriveMotor(SwerveConstants.RLDriveID, SwerveConstants.RLDriveGains, SwerveConstants.RLDriveInverted),
        new SwerveModule.SteerMotor(SwerveConstants.RLSteerID, SwerveConstants.RLSteerGains, SwerveConstants.RLSteerInverted),
        new Translation2d(-SwerveConstants.WHEEL_BASE_METERS / 2, SwerveConstants.TRACK_WIDE_METERS / 2));

    public SwerveModule RRModule = new SwerveModule(
        new SwerveModule.DriveMotor(SwerveConstants.RRDriveID, SwerveConstants.RRDriveGains, SwerveConstants.RRDriveInverted),
        new SwerveModule.SteerMotor(SwerveConstants.RRSteerID, SwerveConstants.RRSteerGains, SwerveConstants.RRSteerInverted),
        new Translation2d(-SwerveConstants.WHEEL_BASE_METERS / 2, -SwerveConstants.TRACK_WIDE_METERS / 2));

    public SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        FLModule.moduleXYTranslation,
        FRModule.moduleXYTranslation,
        RLModule.moduleXYTranslation,
        RRModule.moduleXYTranslation);

    public final List<SwerveModule> swerveModuleList = List.of(
        FLModule,
        FRModule,
        RLModule,
        RRModule);

    public String getSteerMethodStrings() {
        return FLModule.steerMode + FRModule.steerMode + RLModule.steerMode + RRModule.steerMode;
    }

    public void setModuleSpeeds(SwerveModuleState[] _swerveModuleSates) {
        FLModule.setDesiredState(_swerveModuleSates[0]);
        FRModule.setDesiredState(_swerveModuleSates[1]);
        RLModule.setDesiredState(_swerveModuleSates[2]);
        RRModule.setDesiredState(_swerveModuleSates[3]);
    }

    public void setToCoast() {
        if(NeutralMode == NeutralModeValue.Brake &&
        Math.abs(FLModule.driveMotor.getVelocity().getValue()) < 100 &&
        Math.abs(FRModule.driveMotor.getVelocity().getValue()) < 100 &&
        Math.abs(RLModule.driveMotor.getVelocity().getValue()) < 100 &&
        Math.abs(RRModule.driveMotor.getVelocity().getValue()) < 100) {
            FRModule.setModuleToCoast();
            FLModule.setModuleToCoast();
            RLModule.setModuleToCoast();
            RRModule.setModuleToCoast();
            NeutralMode = NeutralModeValue.Coast;
        }
    }

    public void setToBrake() {
        FLModule.setModuleToBrake();
        FRModule.setModuleToBrake();
        RLModule.setModuleToBrake();
        RRModule.setModuleToBrake();
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            FLModule.getPosition(),
            FRModule.getPosition(),
            RLModule.getPosition(),
            RRModule.getPosition()
        };
    }
}
