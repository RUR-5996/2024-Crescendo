package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.swerve.DriveTrain;
import frc.robot.Subsystems.swerve.SwerveDef;
import frc.robot.util.LimelightHelpers;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

@SuppressWarnings("removal")
public class SwerveDrive extends SubsystemBase implements Loggable{

    //control variables
    DriveTrain DRIVETRAIN;

    PIDController tagController;
    PIDController noteController;

    double xSpeed = 0;
    double ySpeed = 0;
    double rotation = 0;
    double holdAngle = 0;
    double defaultHoldAngle = 0;
    double defaultAngle = 0;
    double rotationControllerOutput;
    double deltaTime = 0;
    double prevTime = 0;
    double tagControllerOutput;
    double noteControllerOutput;

    boolean fieldRelative = true; 
    boolean assistedDrive = false;
    boolean rampToggle = false;
    boolean slowmode = false;
    boolean aprilTagDetected = false;
    boolean holdAngleEnabled = false;
    boolean tagControllerEnabled = false;
    boolean noteControllerEnabled = false;

    Pose2d robotPose = new Pose2d();
    Pose2d prevRobotPose = new Pose2d();

    ChassisSpeeds chassisSpeeds;

    SwerveDrivePoseEstimator m_odometry;

    public static AHRS gyro = SwerveDef.gyro;

    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    PIDController angleHoldController = new PIDController(10, 0, 0); // edit the vals

    PIDController rotationController;

    //JoystickButton b8 = new JoystickButton(sController, 1); //TODO presunout do nejaky ButtonMapy
    //JoystickButton b9 = new JoystickButton(sController, 8); //TODO premistit

    //assistPID pid = new assistPID(0.1, 0, 0, 0);

    /**
     * Function for getting the single instance of this class
     * @return SwerveDrive instance
     */
    /*public static SwerveDrive getInstance() {
        if(SWERVE == null) {
            SWERVE = new SwerveDrive();
        }
        return SWERVE;
    }*/

    /**
     * Function for setting up the SwerveDrive object
     */
    public SwerveDrive() {

        DRIVETRAIN = DriveTrain.getInstance();

        gyro.reset();
        gyro.setAngleAdjustment(0);

        m_odometry = new SwerveDrivePoseEstimator(
            DRIVETRAIN.swerveKinematics,
            getHeading(),
            DRIVETRAIN.getModulePositions(),
            robotPose,
            VecBuilder.fill(0.001, 0.001, Units.degreesToRadians(0.1)),
            VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(10)));

        setFieldOriented();
        angleHoldController.disableContinuousInput();
        angleHoldController.setTolerance(Math.toRadians(2)); // the usual drift

        rotationController = new PIDController(
            SwerveConstants.P_ROTATION_CONTROLLER, 
            SwerveConstants.I_ROTATION_CONTROLLER, 
            SwerveConstants.D_ROTATION_CONTROLLER);
        rotationController.enableContinuousInput(-180, 180);
        rotationController.setTolerance(2);

        tagController  = new PIDController(0.1, 0, 0);
        tagController.setTolerance(1);
        noteController = new PIDController(0.1, 0, 0);
        noteController.setTolerance(1);

        chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, gyro.getRotation2d());
    }

    /**
     * Function with the decision tree for driving the chassis
     */
    @Override
    public void periodic() {

        deltaTime = Timer.getFPGATimestamp() - prevTime;
        prevTime = Timer.getFPGATimestamp();

        prevRobotPose = m_odometry.getEstimatedPosition();
        robotPose = updateOdometry();

    }

    public Command configState(String shooterStateName) {
        //String shooterStateName = shooterStateNameSupp.toString();
        return Commands.runOnce(() ->  {
        switch(shooterStateName) {
            case "HOME":
                setHoldAngleFlag(false);
                setNoteControllerFlag(false);
                setTagControllerFlag(false);
                setSlowmodeFlag(false);
                setHoldAngle(defaultAngle);
                break;
            case "INTAKE":
                setHoldAngleFlag(false);
                setNoteControllerFlag(false);
                setTagControllerFlag(false);
                setHoldAngle(defaultAngle);
                setSlowmodeFlag(false);
                break;
            case "LOADING_STATION":
                setHoldAngleFlag(true);
                if(isRed()) {
                    setHoldAngle(-120);
                } else {
                    setHoldAngle(120);
                }
                setNoteControllerFlag(false);
                setTagControllerFlag(false);
                setSlowmodeFlag(true);
                break;
            case "AMP":
                setHoldAngleFlag(true);
                if(isRed()) {
                    setHoldAngle(90);
                } else {
                    setHoldAngle(-90);
                }
                setNoteControllerFlag(false);
                setTagControllerFlag(false);
                setSlowmodeFlag(true);
                break;
            case "SPEAKER_FRONT":
                setHoldAngleFlag(false);
                setNoteControllerFlag(false);
                setTagControllerFlag(false);
                setHoldAngle(defaultAngle);
                setSlowmodeFlag(false);
                break;
            case "SPEAKER_BACK":
                setHoldAngleFlag(false);
                setNoteControllerFlag(false);
                setTagControllerFlag(false);
                setHoldAngle(defaultAngle);
                setSlowmodeFlag(false);
                break;
            case "NULL":
                setHoldAngleFlag(false);
                setNoteControllerFlag(false);
                setTagControllerFlag(false);
                setHoldAngle(defaultAngle);
                setSlowmodeFlag(false);
                break;
            case "CLIMBER":
                setHoldAngleFlag(false);
                setNoteControllerFlag(false);
                setTagControllerFlag(false);
                setSlowmodeFlag(true);
                break;
            case "DEFENSE":
                setHoldAngleFlag(false);
                setNoteControllerFlag(false);
                setTagControllerFlag(false);
                setSlowmodeFlag(false);
                break;
        }});
    }   
    
    public Command toggleSlowMode() {
        return Commands.runOnce(() -> {slowmode = !slowmode;});
    }
    
    public void setSlowmodeFlag(boolean flag) {
        slowmode = flag;
    }

    @Log
    public double updateRotationController() {
        rotationControllerOutput = rotationController.calculate(
            getOdometryDegrees(),
            holdAngle);
        rotationControllerOutput = MathUtil.clamp(rotationControllerOutput, -1, 1);
        return -rotationControllerOutput * SwerveConstants.MAX_SPEED_RADIANSperSECOND;
    }

    @Log
    public double updateTagController() { //TODO disable for champs
        boolean seeTag = LimelightHelpers.getTV("zadni");
        if(seeTag) {
            double tagPosition = LimelightHelpers.getTX("zadni");
            tagControllerOutput = tagController.calculate(tagPosition, 0);
            tagControllerOutput = MathUtil.clamp(tagControllerOutput, -1, 1);
            return tagControllerOutput * SwerveConstants.MAX_SPEED_METERSperSECOND;
        } else {
            return 2;
        }
    }

    @Log
    public double updateNoteController() { 
        //TODO aligns robot in x axis (robot relative). Might want to change to rotation but this should be easier for the drivers since they control only 1 axis (rotation shouldn|t be necessary)
        boolean seeNote = LimelightHelpers.getTV("predni");
        if(seeNote) {
            double notePosition = LimelightHelpers.getTX("predni");
            noteControllerOutput = noteController.calculate(notePosition, 0);
            noteControllerOutput = MathUtil.clamp(noteControllerOutput, -1, 1);
            return noteControllerOutput * SwerveConstants.MAX_SPEED_METERSperSECOND;
        } else {
            return 2;
        }
    }

    public Pose2d updateOdometry() {
        return m_odometry.update(getHeading(), DRIVETRAIN.getModulePositions());
    }

    public void resetOdometry(Pose2d newPose) {
        m_odometry.resetPosition(newPose.getRotation(), DRIVETRAIN.getModulePositions(), newPose);
    }

    public boolean getAtGoal() {
        return rotationController.atSetpoint();
    }

    //@Log
    public boolean getSlowMode() {
        return slowmode;
    }

    @Log
    public double getOdometryDegrees() {
        return getPose().getRotation().getDegrees();
    }

    @Log
    public double getRobotAngleDegrees() {
        return getHeading().getDegrees();
    }  

    public DoubleSupplier supplyRobotAngleDegrees() {
        DoubleSupplier angle = () -> getOdometryDegrees();
        return angle;
    }

    public Command resetGyro() {
        return Commands.runOnce(() -> gyro.reset());
    }

    @Log
    public double getyMeters() {
        return m_odometry.getEstimatedPosition().getY();
    }

    @Log
    public double getxMeters() {
        return m_odometry.getEstimatedPosition().getX();
    }

    @Log
    public double getHoldAngle() {
        return holdAngle;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return chassisSpeeds;
    }

    public Pose2d getOdometryPose() {
        return m_odometry.getEstimatedPosition();
    }

    public Pose2d getPose() {
        return robotPose;
    }
    
    public static Rotation2d getHeading() {
        return gyro.getRotation2d();
    }

    public SwerveDriveKinematics getKinematics() {
        return DRIVETRAIN.swerveKinematics;
    }

    @Log
    public boolean getHoldAngleEnabled() {
        return holdAngleEnabled;
    }

    public void setFieldOriented() {
        fieldRelative = true;
        holdAngle = Math.toRadians(SwerveDef.gyro.getAngle());
    }

    public void setHoldAngle(double angle) { //ve STUPNICH
        holdAngle = angle;
    }

    public void setHoldAngleFlag(boolean flag) {
        holdAngleEnabled = flag;
    }

    public void setNoteControllerFlag(boolean flag) {
        noteControllerEnabled = flag;
    }

    public void setTagControllerFlag(boolean flag) {
        tagControllerEnabled = flag;
    }

    public void setRobotOriented() { //TODO zrusit, NEPOUZIVAT
        fieldRelative = false;
    }

    public void setOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose.getRotation(), modulePositions, pose);
    }

    public void setAutoModuleStates(SwerveModuleState[] states) {
        DRIVETRAIN.setModuleSpeeds(states);
    }

    public void setAutoChassisSpeeds(ChassisSpeeds speeds) {
        chassisSpeeds = speeds;
        setAutoModuleStates(getKinematics().toSwerveModuleStates(speeds));
    }

    public void setToBrake() {
        DRIVETRAIN.setToBrake();
    }

    public void setToCoast() {
        DRIVETRAIN.setToCoast();
    }

    public Command joystickDrive(DoubleSupplier lx, DoubleSupplier ly, DoubleSupplier rx, SwerveDrive drive) {
        return Commands.run(() -> {
            SwerveModuleState[] states;

            double leftX = lx.getAsDouble();
            double leftY = ly.getAsDouble();
            double rightX = rx.getAsDouble();

            xSpeed = leftX * leftX * Math.signum(leftX) * SwerveConstants.MAX_SPEED_METERSperSECOND * DriverConstants.DRIVE_GOVERNOR;
            ySpeed = leftY * leftY * Math.signum(leftY) * SwerveConstants.MAX_SPEED_METERSperSECOND * DriverConstants.DRIVE_GOVERNOR;

            if(slowmode) { //TODO probably disable, there is no need to go slow this year and there is nothing to break on the robot
                xSpeed = xSpeed * DriverConstants.PRECISION_RATIO;
                ySpeed = ySpeed * DriverConstants.PRECISION_RATIO;
                //rotation = rotation * DriverConstants.PRECISION_RATIO;
            }

            if(holdAngleEnabled) {
                rotation = updateRotationController();
                System.out.println("setting");
            }
            else {
                rotation = rightX * SwerveConstants.MAX_SPEED_RADIANSperSECOND * DriverConstants.TURN_GOVERNOR;
            }

            if(noteControllerEnabled) { //TODO test and potentialy disable
                double newXSpeed = updateNoteController(); //TODO test with drivers and Notes. In theory this aligns the robot as long as the tag is visible. Then returns X axis to drivers
                if(newXSpeed <= 1) { //Once noteController is enabled, the robot goes into robot relative mode to mitigate any robot rotation
                    chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(ySpeed, newXSpeed, rotation, gyro.getRotation2d());
                    states = DRIVETRAIN.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
                } else { //TODO might want to change the xSpeed parameter to 0 to disable any driver input in an already aligned axis
                    chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(ySpeed, xSpeed, rotation, gyro.getRotation2d());
                    states = DRIVETRAIN.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
                }
                
            } else if(tagControllerEnabled) { //TODO TEST and force the drivers to use this
                double newXSpeed = updateTagController();
                if(newXSpeed <= 1) {
                    chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(ySpeed, newXSpeed, rotation, gyro.getRotation2d());
                    states = DRIVETRAIN.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
                } else { //TODO maybe replace xSpeed with 0
                    chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(ySpeed, xSpeed, rotation, gyro.getRotation2d());
                    states = DRIVETRAIN.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
                }
            } else {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, rotation, gyro.getRotation2d());
                states = DRIVETRAIN.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
            }

            SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED_METERSperSECOND);

            DRIVETRAIN.setModuleSpeeds(states);
        }, drive);
    }

    public double deadzone(double input) { //TODO prepsat inline
        if (Math.abs(input) < 0.2) {
            return 0;
        } else {
            return input;
        }
    }

    public boolean isRed() {
        return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    //@Config.ToggleButton(defaultValue = false, tabName = "robotMain", columnIndex = 2, rowIndex = 0, width = 2, height = 2)
    void gyroReset(boolean input) {
        if(input) {
            m_odometry.resetPosition(getHeading(), DRIVETRAIN.getModulePositions(), new Pose2d(robotPose.getTranslation(), new Rotation2d(0)));
        }
    }

    static class assistPID extends PIDController {
        double setpoint = 0;
        double maxSpeed = 0;
        double offset = 0;
        public assistPID(double kP, double kI, double kD, double setpoint) {
            super(kP, kI, kD);
            this.setpoint = setpoint;
            this.maxSpeed = SwerveConstants.MAX_SPEED_METERSperSECOND * DriverConstants.DRIVE_GOVERNOR;
        }

        public void setOffset(double value) {
            offset = value;
        }

        public double pidGet() {
            double speed = MathUtil.clamp(super.calculate(offset, setpoint), -maxSpeed, maxSpeed);
            return -speed;
        }
    }
}