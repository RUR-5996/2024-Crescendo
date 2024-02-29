package frc.robot.Subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class SwerveModule {

    private StatusSignal<Double> m_drivePosition;
    private StatusSignal<Double> m_driveVelocity;
    private BaseStatusSignal[] m_talonSignals;
    private double m_steerPosition;
    private double m_steerVelocity;
    private double[] m_sparkSignals;
    private double m_driveRotationsPerMeter = 0;

    TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration();
    private SwerveModulePosition m_internalState = new SwerveModulePosition();
    private VelocityTorqueCurrentFOC m_velocitySetter = new VelocityTorqueCurrentFOC(0);

    public final SteerMotor steerMotor;
    public final DriveMotor driveMotor;
    public final Translation2d moduleXYTranslation;
    public String steerMode = "integrated";
    public boolean hasSwerveSeedingOccured = false;
    public boolean hasCANCoderBeenSetToAbs = false; //unused
    public double swerveSeedingRetryCount = 0;

    public static SimpleMotorFeedforward driveMotorFeedForward = new SimpleMotorFeedforward(SwerveConstants.kS, SwerveConstants.kV, SwerveConstants.kA);

    public SwerveModule(DriveMotor driveMotor, SteerMotor steerMotor, Translation2d moduleXYTranslation) {
        this.steerMotor = steerMotor;
        this.driveMotor = driveMotor;
        this.moduleXYTranslation = moduleXYTranslation;

        if(RobotBase.isSimulation()) {
            //simModule = new SwerveModuleSim(driveMotor, steerMotor); //do we need this?
        } else {
            //simModule = null;
        }

        swerveModuleInit();
    }

    private void swerveModuleInit() {
        driveMotor.getConfigurator().refresh(driveTalonConfig);
        final Slot0Configs DriveMotorGains = new Slot0Configs();
        
        DriveMotorGains.kP = SwerveConstants.kP;
        DriveMotorGains.kI = 0;
        DriveMotorGains.kD = 0;
        DriveMotorGains.kS = SwerveConstants.kS;
        DriveMotorGains.kV = SwerveConstants.kV;

        driveTalonConfig.Slot0 = DriveMotorGains;
        driveTalonConfig.MotorOutput.Inverted = driveMotor.kInverted;
        driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        driveMotor.getConfigurator().apply(driveTalonConfig);

        steerMotor.restoreFactoryDefaults();
        steerMotor.setInverted(steerMotor.kInverted);
        steerMotor.setIdleMode(IdleMode.kBrake);
        steerMotor.steerEncoder.setPositionConversionFactor(SwerveConstants.kSteerConversionFactor);
        steerMotor.steerEncoder.setInverted(steerMotor.kInverted); //might need separate variable
        steerMotor.testEncoder.setInverted(steerMotor.kInverted);
        steerMotor.testEncoder.setPositionConversionFactor(SwerveConstants.kSteerConversionFactor);
        steerMotor.m_PidController.setP(steerMotor.kGains.kP);
        steerMotor.m_PidController.setI(steerMotor.kGains.kI);
        steerMotor.m_PidController.setD(steerMotor.kGains.kD);
        steerMotor.m_PidController.setIZone(steerMotor.kGains.kIzone);
        steerMotor.m_PidController.setOutputRange(-steerMotor.kGains.kPeakOutput, steerMotor.kGains.kPeakOutput);
        steerMotor.m_PidController.setFeedbackDevice(steerMotor.steerEncoder);

        m_drivePosition = driveMotor.getPosition();
        m_driveVelocity = driveMotor.getVelocity();
        m_steerPosition = steerMotor.steerEncoder.getPosition();
        m_steerVelocity = steerMotor.steerEncoder.getVelocity();

        m_talonSignals = new BaseStatusSignal[2];
        m_talonSignals[0] = m_drivePosition;
        m_talonSignals[1] = m_driveVelocity;

        m_sparkSignals = new double[2];
        m_sparkSignals[0] = m_steerPosition;
        m_sparkSignals[1] = m_steerVelocity;

        double rotationsPerWheelRotation = SwerveConstants.DRIVE_MOTOR_GEARING;
        double metersPerWheelRotation = 2*Math.PI*SwerveConstants.WHEEL_RADIUS_METERS;
        m_driveRotationsPerMeter = rotationsPerWheelRotation/metersPerWheelRotation;
    }

    public void setModuleToCoast() {
        driveMotor.getConfigurator().refresh(driveTalonConfig);
        driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(driveTalonConfig);

        steerMotor.setIdleMode(IdleMode.kCoast);
    }

    public void setModuleToBrake() {
        driveMotor.getConfigurator().refresh(driveTalonConfig);
        driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotor.getConfigurator().apply(driveTalonConfig);

        steerMotor.setIdleMode(IdleMode.kBrake);
    }

    public SwerveModulePosition getPosition() {
        m_drivePosition.refresh();
        m_driveVelocity.refresh();
        //m_steerPosition = steerMotor.steerEncoder.getPosition();
        m_steerPosition = steerMotor.testEncoder.getPosition();
        m_steerVelocity = steerMotor.steerEncoder.getVelocity();

        double drive_rot = m_drivePosition.getValue() + (m_driveVelocity.getValue() * m_drivePosition.getTimestamp().getLatency());
        double angle_rot = m_steerPosition;
        m_internalState.distanceMeters = drive_rot / m_driveRotationsPerMeter;
        m_internalState.angle = Rotation2d.fromRotations(angle_rot);

        return m_internalState;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        var optimized = SwerveModuleState.optimize(desiredState, m_internalState.angle);

        double angleToSet = optimized.angle.getRotations();
        steerMotor.m_PidController.setReference(angleToSet, CANSparkMax.ControlType.kPosition);//snad

        double velocityToSet = optimized.speedMetersPerSecond * m_driveRotationsPerMeter;
        driveMotor.setControl(m_velocitySetter.withVelocity(velocityToSet));
    }

    public static class SteerMotor extends CANSparkMax {
        public SwerveConstants.Gains kGains;
        public boolean kInverted;
        public RelativeEncoder steerEncoder;
        public RelativeEncoder testEncoder; //rename if works probably not
        public double kOffsetDegrees;
        public SparkPIDController m_PidController;

        public SteerMotor(int _motorID, SwerveConstants.Gains _gains, boolean _inverted) {
            super(_motorID, MotorType.kBrushless);
            kGains = _gains;
            kInverted = _inverted;
            steerEncoder = super.getEncoder();
            //testEncoder = super.getAbsoluteEncoder();
            testEncoder = super.getEncoder();
            kOffsetDegrees = 0;
            m_PidController = super.getPIDController();
        }

        public SteerMotor(int _motorID, SwerveConstants.Gains _gains, boolean _inverted, double _offsetDegrees) {
            super(_motorID, MotorType.kBrushless);
            kGains = _gains;
            kInverted = _inverted;
            steerEncoder = super.getEncoder();
            testEncoder = super.getEncoder();
            kOffsetDegrees = _offsetDegrees;
            m_PidController = super.getPIDController();
        }
    }

    public static class DriveMotor extends TalonFX {
        public SwerveConstants.Gains kGains;
        public InvertedValue kInverted;

        public DriveMotor(int _motorID, SwerveConstants.Gains _gains, InvertedValue _inverted) {
            super(_motorID);
            kGains = _gains;
            kInverted = _inverted;
        }
    }
    //add absolute encoder for homing later on
}