package frc.robot.Subsystems;

import frc.robot.Constants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Subsystems.swerve.SwerveConstants;
import frc.robot.Subsystems.swerve.SwerveDriveDef;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightPose2d;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class SwerveDrive extends SubsystemBase {
    SwerveDriveDef m_driveTrain;
    Pose2d prevRobotPose = new Pose2d();
    Pose2d robotPose = new Pose2d();
    Pose2d visionPoseFront = new Pose2d();
    double deltaTime = 0;
    double prevTime = 0;
    AHRS gyro;
    SwerveDrivePoseEstimator m_odometry;
    double holdAngle = 0;
    boolean holdAngleEnabled = false;
    ProfiledPIDController rotationController, rollRotationController, pitchRotationController;
    double rotationControllerOutput;
    SlewRateLimiter xLimiter, yLimiter, rotLimiter;
    ChassisSpeeds chassisSpeeds;

    boolean aprilTagDetected = false;

    public Field2d m_field;
    public double [] akitPose = {0, 0, 0};
    private static SwerveDrive instance;

    private SwerveDrive() {
        //might be for ramp 2023??
        rotLimiter = new SlewRateLimiter(4.5*4*2); //??
        xLimiter = new SlewRateLimiter(60, -1000000000, 0);
        yLimiter = new SlewRateLimiter(60, -1000000000, 0);
        rollRotationController = new ProfiledPIDController(5.4/67, 0, 0, new TrapezoidProfile.Constraints(.7,.7));
        pitchRotationController = new ProfiledPIDController(5.4/67-.02, 0, 5.4/680*1.5, new TrapezoidProfile.Constraints(.7,.7));
        rotationController = new ProfiledPIDController(
            SwerveConstants.P_ROTATION_CONTROLLER, 
            SwerveConstants.I_ROTATION_CONTROLLER, 
            SwerveConstants.D_ROTATION_CONTROLLER, 
            new TrapezoidProfile.Constraints(SwerveConstants.MAX_SPEED_RADIANSperSECOND, 5*Units.radiansToDegrees(SwerveConstants.MAX_SPEED_RADIANSperSECOND)));
        rotationController.enableContinuousInput(-180, 180);
        rotationController.setTolerance(2); //TODO might be a lot
        gyro = new AHRS(SPI.Port.kMXP);
        gyro.reset();
        gyro.setAngleAdjustment(0);

        m_driveTrain = new SwerveDriveDef();
        m_field = new Field2d();

        m_odometry = new SwerveDrivePoseEstimator(
            m_driveTrain.m_kinematics,
            getRobotAngle(),
            m_driveTrain.getModulePositions(),
            robotPose,
            VecBuilder.fill(0.001, 0.001, Units.degreesToRadians(0.1)),
            VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(10)));

    }

    @Override
    public void periodic() {
        prevRobotPose = m_odometry.getEstimatedPosition();
        deltaTime = Timer.getFPGATimestamp() - prevTime;
        prevTime = Timer.getFPGATimestamp();

        robotPose = updateOdometry();
        //drawRobotOnField(m_field);
    }

    public static SwerveDrive getInstance() {
        if(instance == null) {
            instance = new SwerveDrive();
        }
        return instance;
    }

    public Command joystickDriveCommand(DoubleSupplier _x, DoubleSupplier _y, DoubleSupplier _rot) {
        return Commands.run(
            () -> {
                double x = -(Math.abs(_x.getAsDouble()) < .1 ? 0: _x.getAsDouble());
                double y = -(Math.abs(_y.getAsDouble()) < .1 ? 0: _y.getAsDouble());
                double rot = -Math.pow((Math.abs(_rot.getAsDouble()) < .1 ? 0: _rot.getAsDouble()), 3);
                double joystickDriveGovernor = DriverConstants.DRIVE_GOVERNOR;
                if(DriverConstants.ACCELERATED_INPUTS) {

                } else {
                    x = Math.signum(x) * Math.sqrt(Math.abs(x));
                    y = Math.signum(y) * Math.sqrt(Math.abs(y));
                    rot = Math.signum(rot) * Math.sqrt(Math.abs(rot));
                }
                setDriveSpeeds(new Translation2d(
                    convertToMetersPerSecond(x)*joystickDriveGovernor,
                    convertToMetersPerSecond(y)*joystickDriveGovernor),
                    holdAngleEnabled ? updateRotationController() : convertToRadiansPerSecond(rot) * joystickDriveGovernor,
                    Constants.DriverConstants.FIELD_RELATIVE); 
            }, this);
    }

    public Command testCommand() {
        return Commands.run(() -> {
            m_driveTrain.handBrakeX();
        }
        , this);
    }

    public double updateRotationController() {
        rotationControllerOutput = rotationController.calculate(
            robotPose.getRotation().getRadians(),
            new State(holdAngle, 0.0));
        return rotationControllerOutput;
    }

    public boolean getAtGoal() {
        return rotationController.atGoal();
    }

    public Rotation2d getRobotAngle() {
        return new Rotation2d(Math.toRadians(gyro.getAngle())); //TODO check if this works
    }

    public Pose2d getPose() {
        return robotPose;
    }

    @Log
    public double odometryDegrees() {
        return getPose().getRotation().getDegrees();
    }

    public double getRobotAngleDegrees() {
        return getRobotAngle().getDegrees();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return chassisSpeeds;
    }

    public void setDriveSpeeds(Translation2d xySpeedsMetersPerSec, double rRadiansPerSec, boolean fieldRelative) {
        chassisSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xySpeedsMetersPerSec.getX(),
                xySpeedsMetersPerSec.getY(),
                rRadiansPerSec,
                robotPose.getRotation()) : new ChassisSpeeds(
                    xySpeedsMetersPerSec.getX(),
                    xySpeedsMetersPerSec.getY(),
                    rRadiansPerSec);
        SwerveModuleState [] swerveModuleStates = m_driveTrain.m_kinematics.toSwerveModuleStates(chassisSpeeds);
        
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_SPEED_METERSperSECOND);
        m_driveTrain.setModuleSpeeds(swerveModuleStates);
    }

    public Pose2d updateOdometry() {
        return m_odometry.update(getRobotAngle(), m_driveTrain.getModulePositions());
    }

    public Pose2d getOdometryPose() {
        return m_odometry.getEstimatedPosition();
    }

    public void setHoldAngleFlag(boolean input) {
        holdAngleEnabled = input;
    }

    public boolean getHoldAngleFlag() {
        return holdAngleEnabled;
    }

    public void setHoldAngle(double input) {
        holdAngle = input;
    }

    public void resetOdometry(Pose2d newPose) {
        m_odometry.resetPosition(newPose.getRotation(), m_driveTrain.getModulePositions(), newPose);
    }

    public SwerveDriveKinematics getKinematics() {
        return m_driveTrain.m_kinematics;
    }

    public void setAutoModuleStates(SwerveModuleState[] states) {
        m_driveTrain.setModuleSpeeds(states);
    }

    public void setAutoChassisSpeeds(ChassisSpeeds speeds) {
        setAutoModuleStates(getKinematics().toSwerveModuleStates(speeds));
    }

    public void drawRobotOnField(Field2d field) {
        Pose2d robotPose = m_odometry.getEstimatedPosition();
        if(DriverStation.getAlliance().equals(Alliance.Red)) {
            robotPose = new Pose2d(new Translation2d(16.541748984 - robotPose.getX(), 8.01367968 - robotPose.getY()), robotPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
        }

        field.setRobotPose(robotPose);

        field.getObject("frontLeft").setPose(
            robotPose.transformBy(new Transform2d(m_driveTrain.FLModule.moduleXYTranslation, m_driveTrain.FLModule.getPosition().angle)));
        field.getObject("frontRight").setPose(
            robotPose.transformBy(new Transform2d(m_driveTrain.FRModule.moduleXYTranslation, m_driveTrain.FRModule.getPosition().angle)));
        field.getObject("backLeft").setPose(
            robotPose.transformBy(new Transform2d(m_driveTrain.RLModule.moduleXYTranslation, m_driveTrain.RLModule.getPosition().angle)));
        field.getObject("backRight").setPose(
            robotPose.transformBy(new Transform2d(m_driveTrain.RRModule.moduleXYTranslation, m_driveTrain.RRModule.getPosition().angle)));

        akitPose[0] = this.robotPose.getX();
        akitPose[1] = this.robotPose.getY();
        akitPose[2] = this.robotPose.getRotation().getRadians();
        SmartDashboard.putNumberArray("akitPose", akitPose);    
    }

    public void setToBrake() {
        m_driveTrain.setToBrake();
    }

    public void setToCoast() {
        m_driveTrain.setToCoast();
    }

    public double xMeters() {
        return m_odometry.getEstimatedPosition().getX();
    }

    public double yMeters() {
        return m_odometry.getEstimatedPosition().getY();
    }

    private double convertToMetersPerSecond(double _input) {
        return _input*SwerveConstants.MAX_SPEED_METERSperSECOND;
    }

    private double convertToRadiansPerSecond(double _input) {
        return _input*SwerveConstants.MAX_SPEED_RADIANSperSECOND;
    }

    private LimelightPose2d getLimelightPose(String limelightName) {
        Pose2d pose;
        LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
        if(DriverStation.getAlliance().equals(Alliance.Blue)) {
            pose = results.targetingResults.getBotPose2d_wpiBlue();
        } else {
            pose = results.targetingResults.getBotPose2d_wpiRed();
        }
        double latency = results.targetingResults.latency_jsonParse + results.targetingResults.latency_capture + results.targetingResults.latency_pipeline;
        int aprilTagAmount = results.targetingResults.targets_Fiducials.length;
        return new LimelightPose2d(pose, latency, aprilTagAmount);
    }

    private void limelightOdometry(String limelightName) { //seems unused
        LimelightPose2d llPose = getLimelightPose(limelightName);
        if(llPose.latency < 120 && llPose.getTranslation().getX() > 0 && llPose.getTranslation().getX() < 4.6) {
            if(llPose.aprilTagAmount > 1) {
                double dist = llPose.minus(robotPose).getTranslation().getNorm();
                if(dist > 0.1) {

                }
            }
        }
    }

    @Config.ToggleButton(defaultValue = false, tabName = "nodeSelector", columnIndex = 22, rowIndex = 0, width = 2, height = 2)
    public void resetToGyro(boolean input) {
        if(input) {
            m_odometry.resetPosition(getRobotAngle(), m_driveTrain.getModulePositions(), new Pose2d(robotPose.getTranslation(), new Rotation2d(0)));
        }
    }

    public void activateHandBrake() {
        m_driveTrain.handBrakeX(); //TODO test also handBrake90
    }
}
