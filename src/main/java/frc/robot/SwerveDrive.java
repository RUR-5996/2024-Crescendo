package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.SwerveConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class SwerveDrive implements Loggable{

    //single instance
    private static SwerveDrive SWERVE;

    //control variables
    XboxController controller = Robot.controller;
    GenericHID sController = Robot.sController;
    GenericHID tController = Robot.tController;
    DriveTrain DRIVETRAIN;
    Shooter SHOOTER;

    double xSpeed = 0;
    double ySpeed = 0;
    double rotation = 0;
    double holdAngle = 0;
    double defaultAngle = 0;
    double rotationControllerOutput;
    double deltaTime = 0;
    double prevTime = 0;

    boolean fieldRelative = true; 
    boolean assistedDrive = false;
    boolean rampToggle = false;
    boolean slowmode = false;
    boolean aprilTagDetected = false;
    boolean holdAngleEnabled = false;

    Pose2d robotPose = new Pose2d();
    Pose2d prevRobotPose = new Pose2d();

    ChassisSpeeds chassisSpeeds;

    SwerveDrivePoseEstimator m_odometry;

    static AHRS gyro = SwerveDef.gyro;

    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    PIDController angleHoldController = new PIDController(0.01, 0, 0); // edit the vals

    PIDController rotationController;

    JoystickButton b8 = new JoystickButton(sController, 1); //TODO presunout do nejaky ButtonMapy
    JoystickButton b9 = new JoystickButton(tController, 3);

    //assistPID pid = new assistPID(0.1, 0, 0, 0);

    /**
     * Function for getting the single instance of this class
     * @return SwerveDrive instance
     */
    public static SwerveDrive getInstance() {
        if(SWERVE == null) {
            SWERVE = new SwerveDrive();
        }
        return SWERVE;
    }

    /**
     * Function for setting up the SwerveDrive object
     */
    public SwerveDrive() {

        DRIVETRAIN = DriveTrain.getInstance();
        SHOOTER = Shooter.getInstance();

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
    }

    /**
     * Function with the decision tree for driving the chassis
     */
    public void periodic() {

        orientedDrive();

        deltaTime = Timer.getFPGATimestamp() - prevTime;
        prevTime = Timer.getFPGATimestamp();

        prevRobotPose = m_odometry.getEstimatedPosition();
        robotPose = updateOdometry();

        if(b8.getAsBoolean()) { //cte State ze Shooteru a nastavuje podle toho hodnoty v drivu
            configState(); //TODO pridat blbosti jako APRIL TAG lock
        }

        if(controller.getBButtonReleased()) {
            slowmode = !slowmode; //TODO prepsat setter funkci kvuli logovani
        }

    }

    void configState() {
        switch (SHOOTER.state) {
            case HOME:
                setHoldAngleFlag(false);
                break;
            case INTAKE:
                setHoldAngleFlag(false);
                break;
            case LOADING_STATION:
                setHoldAngleFlag(true);
                setHoldAngle(-120); //TODO zkontrolovat, jestli + nebo -
                break;
            case SHORT_SPEAKER:
                setHoldAngleFlag(false);
                break;
            case AMP:
                setHoldAngleFlag(true);
                setHoldAngle(90); //TODO zkontrolovat, jestli + nebo -
                break;
            case LONG_SPEAKER_FRONT:
                setHoldAngleFlag(false);
                break;
            case LONG_SPEAKER:
                setHoldAngleFlag(false);
                break;
        }
    }

    @Log
    public double updateRotationController() {
        rotationControllerOutput = rotationController.calculate(
            getOdometryDegrees(),
            holdAngle);
        rotationControllerOutput = MathUtil.clamp(rotationControllerOutput, -1, 1);
        return -rotationControllerOutput * SwerveConstants.MAX_SPEED_RADPS;
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

    @Log
    public boolean getPrecisionMode() {
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
        setAutoModuleStates(getKinematics().toSwerveModuleStates(speeds));
    }

    public void setToBrake() {
        DRIVETRAIN.setToBrake();
    }

    public void setToCoast() {
        DRIVETRAIN.setToCoast();
    }

    /* 
    public void drive() {
        
        xSpeed = deadzone(controller.getLeftX()) * SwerveConstants.MAX_SPEED_MPS * DriverConstants.DRIVE_GOVERNOR;
        ySpeed = deadzone(controller.getLeftY()) * SwerveConstants.MAX_SPEED_MPS * DriverConstants.DRIVE_GOVERNOR;
        rotation = deadzone(controller.getRightX()) * SwerveConstants.MAX_SPEED_RADPS * DriverConstants.TURN_GOVERNOR; 

        SwerveModuleState[] states = DRIVETRAIN.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(ySpeed, xSpeed, rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED_MPS);

        DRIVETRAIN.setModuleSpeeds(states);
    }*/

    /*
    public void drive(double xSpeed, double ySpeed, double rotation) { //TODO upravit do jedny velky drive funkce, kde bude i field oriented
        double leftX = deadzone(controller.getLeftX());
        double leftY = deadzone(controller.getLeftY());
        double rightX = deadzone(controller.getRightX());

        xSpeed = leftX * leftX * Math.signum(leftX) * SwerveConstants.MAX_SPEED_MPS * DriverConstants.DRIVE_GOVERNOR;
        ySpeed = leftY * leftY * Math.signum(leftY) * SwerveConstants.MAX_SPEED_MPS * DriverConstants.DRIVE_GOVERNOR;

        if(holdAngleEnabled) {
            rotation = updateRotationController();
        } else {
            rotation = rightX * rightX * Math.signum(rightX) * SwerveConstants.MAX_SPEED_RADPS * DriverConstants.TURN_GOVERNOR;
        }

        if(slowmode) {
            xSpeed /= 3;
            ySpeed /= 3;
            rotation /= 3;
        }

        SwerveModuleState[] states = DRIVETRAIN.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED_MPS);

        DRIVETRAIN.setModuleSpeeds(states);
    }*/

    public void orientedDrive() {

        double leftX = deadzone(controller.getLeftX());
        double leftY = deadzone(controller.getLeftY());
        double rightX = deadzone(controller.getRightX());

        xSpeed = leftX * leftX * Math.signum(leftX) * SwerveConstants.MAX_SPEED_MPS * DriverConstants.DRIVE_GOVERNOR;
        ySpeed = leftY * leftY * Math.signum(leftY) * SwerveConstants.MAX_SPEED_MPS * DriverConstants.DRIVE_GOVERNOR;

        if(holdAngleEnabled) {
            rotation = updateRotationController();
        }
        else {
            rotation = rightX * SwerveConstants.MAX_SPEED_RADPS * DriverConstants.TURN_GOVERNOR;
        }

        if(slowmode) {
            xSpeed = xSpeed * DriverConstants.PRECISION_RATIO;
            ySpeed = ySpeed * DriverConstants.PRECISION_RATIO;
            rotation = rotation * DriverConstants.PRECISION_RATIO;
        }

        SwerveModuleState[] states = DRIVETRAIN.swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, rotation, gyro.getRotation2d()));

        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED_MPS); //konecne funguje

        DRIVETRAIN.setModuleSpeeds(states);
    }

    /**public static void assistedDrive() {
        pid.setOffset(LimelightAiming.tapeLimelight1.X);

        ySpeed = deadzone(controller.getLeftY()) * SwerveDef.MAX_SPEED_MPS * SwerveDef.DRIVE_COEFFICIENT;
        xSpeed = pid.pidGet();
        rotation = deadzone(controller.getRightX()) * SwerveDef.MAX_SPEED_RADPS * SwerveDef.TURN_COEFFICIENT;

        SwerveModuleState[] states = swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, rotation, SwerveDef.gyro.getRotation2d()));

        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveDef.MAX_SPEED_MPS);

        SwerveDef.flModule.setState(states[0]);
        SwerveDef.frModule.setState(states[1]);
        SwerveDef.rlModule.setState(states[2]);
        SwerveDef.rrModule.setState(states[3]);
    }*/

    public double deadzone(double input) { //TODO prepsat inline
        if (Math.abs(input) < 0.2) {
            return 0;
        } else {
            return input;
        }
    }

    @Config.ToggleButton(defaultValue = false, tabName = "robotMain", columnIndex = 2, rowIndex = 0, width = 2, height = 2)
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
            this.maxSpeed = SwerveConstants.MAX_SPEED_MPS * DriverConstants.DRIVE_GOVERNOR;
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