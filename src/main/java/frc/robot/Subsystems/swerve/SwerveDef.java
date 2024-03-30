package frc.robot.Subsystems.swerve;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class SwerveDef implements Loggable {

    public static final AHRS gyro = new AHRS(SPI.Port.kMXP);

    public static class SteerMotor extends CANSparkMax {
        public boolean inverted;

        public SteerMotor(int id, boolean inverted) {
            super(id, CANSparkMax.MotorType.kBrushless);
            this.inverted = inverted;
        }
    }

    public static class DriveMotor extends TalonFX {
        public InvertedValue invertType;

        public DriveMotor(int id, InvertedValue invertType) {
            super(id);
            this.invertType = invertType;
        }
    }

    // Use this structure when using off-motor encoder. Implement in SwerveModule
    // class!
    public static class SteerSensor extends AnalogInput {
        public double offset;
        public AnalogInput sensor;

        public SteerSensor(int id, double offset) {
            super(id);
            this.offset = offset;
        }
    }

    // should create a lib for this
    public static class SwerveModule {
        public RelativeEncoder neoEncoder;
        public SparkAbsoluteEncoder neoAbsEncoder;
        public SparkPIDController neoController;
        public SteerMotor steerMotor;
        public DriveMotor driveMotor;
        public SteerSensor steerSensor;
        public pidValues drivePID, steerPID;
        public initPID turnPID;
        private TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration();
        private VelocityVoltage m_velocitySetter = new VelocityVoltage(0);
        double rotationsPerWheelRotation = SwerveConstants.DRIVE_MOTOR_GEARING;
        double metersPerWheelRotation = 2*Math.PI*SwerveConstants.WHEEL_RADIUS_METERS;
        double m_driveRotationsPerMeter = rotationsPerWheelRotation/metersPerWheelRotation;
        SwerveModulePosition position = new SwerveModulePosition();

        public SwerveModule(SteerMotor sMotor, pidValues sPID, DriveMotor dMotor, pidValues dPID, SteerSensor sensor) {
            steerMotor = sMotor;
            driveMotor = dMotor;
            drivePID = dPID;
            steerPID = sPID;
            steerSensor = sensor;
            turnPID = new initPID(steerPID.kP, steerPID.kI, steerPID.kD, 1, 0);
            neoController = steerMotor.getPIDController();
            neoEncoder = steerMotor.getEncoder(); //should return NEO built-in encoder
        }

        public void moduleInit() {

            driveMotor.getConfigurator().refresh(driveTalonConfig);
            final Slot0Configs DriveMotorGains = new Slot0Configs();
        
            DriveMotorGains.kP = SwerveConstants.kP;
            DriveMotorGains.kI = 0;
            DriveMotorGains.kD = 0;

            driveTalonConfig.Slot0 = DriveMotorGains;
            OpenLoopRampsConfigs openLoopConfig = new OpenLoopRampsConfigs();
            openLoopConfig.TorqueOpenLoopRampPeriod = 0.3;
            openLoopConfig.DutyCycleOpenLoopRampPeriod = 0.3;
            openLoopConfig.VoltageOpenLoopRampPeriod = 0.3;
            ClosedLoopRampsConfigs closedLoopConfig = new ClosedLoopRampsConfigs();
            closedLoopConfig.VoltageClosedLoopRampPeriod = 0.3;
            closedLoopConfig.TorqueClosedLoopRampPeriod = 0.3;
            closedLoopConfig.DutyCycleClosedLoopRampPeriod = 0.3;
            driveTalonConfig.OpenLoopRamps = openLoopConfig;
            driveTalonConfig.ClosedLoopRamps = closedLoopConfig;
            driveTalonConfig.MotorOutput.Inverted = driveMotor.invertType;
            driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            driveMotor.getConfigurator().apply(driveTalonConfig);

            steerMotor.restoreFactoryDefaults();
            steerMotor.setInverted(steerMotor.inverted);
            steerMotor.setOpenLoopRampRate(0.2);
            steerMotor.setClosedLoopRampRate(0.2);

            neoEncoder.setPositionConversionFactor(SwerveConstants.STEER_FEEDBACK_COEFFICIENT);

            neoController.setP(steerPID.kP);
            neoController.setI(steerPID.kI);
            neoController.setD(steerPID.kD);
            neoController.setFF(steerPID.kF);
            neoController.setIZone(300);
            neoController.setOutputRange(-1, 1);
            neoController.setFeedbackDevice(neoEncoder);
            //4zeroEncoder();
        }

        public void setToCoast() {
            driveMotor.getConfigurator().refresh(driveTalonConfig);
            driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            driveMotor.getConfigurator().apply(driveTalonConfig);

            steerMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        }

        public void setToBrake() {
            driveMotor.getConfigurator().refresh(driveTalonConfig);
            driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            driveMotor.getConfigurator().apply(driveTalonConfig);

            steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        }

        public void zeroEncoder() {
            neoEncoder.setPosition(0);
        }

        public SwerveModulePosition getState() {
            double drive_rot = driveMotor.getPosition().getValue() + (driveMotor.getVelocity().getValue() * driveMotor.getPosition().getTimestamp().getLatency());
            double angle = neoEncoder.getPosition(); //degrees
            position.distanceMeters = drive_rot / m_driveRotationsPerMeter;
            position.angle = new Rotation2d(Math.toRadians(angle));
            return position;
            //Rotation2d rotation = new Rotation2d(Math.toRadians(neoEncoder.getPosition()));
            //return new SwerveModulePosition(speed, rotation);
        }
        /**
         * @deprecated included in new WPILib
         * @param stateToOptimize
         * @param moduleAngle
         * @return
         */
        public SwerveModuleState optimizeState(SwerveModuleState stateToOptimize, Rotation2d moduleAngle) {
            Rotation2d diff = stateToOptimize.angle.minus(moduleAngle);

            if (Math.abs(diff.getDegrees()) > 90) {
                return new SwerveModuleState(-stateToOptimize.speedMetersPerSecond, stateToOptimize.angle.rotateBy(Rotation2d.fromDegrees(180)));
            } else {
                return new SwerveModuleState(stateToOptimize.speedMetersPerSecond, stateToOptimize.angle);
            }
        }

        public void setAngle(double angleToSet) {
            double encoderPosition = neoEncoder.getPosition(); //degrees, can be over 360
            double toFullCircle = Math.IEEEremainder(encoderPosition, 360);
            double newAngle = angleToSet + encoderPosition - toFullCircle;
            
            if (newAngle - encoderPosition > 180) {
                newAngle -= 360;
            } else if (newAngle - encoderPosition < -180) {
                newAngle += 360;
            }
            
            neoController.setReference(newAngle, CANSparkMax.ControlType.kPosition); // runs PID with adjusted angle over 360 degs
        }

        public void setState(SwerveModuleState stateToSet) {
            //SwerveModuleState optimizedState = SwerveModuleState.optimize(stateToSet, new Rotation2d(Math.toRadians(neoEncoder.getPosition())));
            SwerveModuleState optimizedState = optimizeState(stateToSet, new Rotation2d(Math.toRadians(neoEncoder.getPosition())));
            //SwerveModuleState optimizedState = stateToSet;

            setAngle(optimizedState.angle.getDegrees());
            
            //neoController.setReference(optimizedState.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
            //turnPID.setOffset(clampContinuousDegs(neoEncoder.getPosition()));
            //turnPID.setTarget(optimizedState.angle.getDegrees());
            //steerMotor.set(turnPID.pidGet());

            SmartDashboard.putNumber("speed", optimizedState.speedMetersPerSecond * m_driveRotationsPerMeter);
            driveMotor.setControl(m_velocitySetter.withVelocity(optimizedState.speedMetersPerSecond * m_driveRotationsPerMeter));
        }

        @Log
        public double getNeoAngle() {
            return neoEncoder.getPosition();
        }

        public double clampContinuousDegs(double toClamp) {
            if (toClamp < -180) {
                return 180-(toClamp%180.0);
            } else if (toClamp > 180) {
                return -180+(toClamp%180);
            } else {
                return toClamp;
            }
        }

    }

    // compact way to stiore PID controller constants
    public static class pidValues {
        public double kP;
        public double kI;
        public double kD;
        public double kF;

        public pidValues(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = 0;
        }

        public pidValues(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        }
    }

    public static class initPID extends PIDController {
        private double target, offset, maxSpeed;

        public initPID(double kP, double kI, double kD, double mSpeed, double setpoint) {
            super(kP, kI, kD);
            super.setTolerance(0.72);
            super.enableContinuousInput(-180, 180);
            maxSpeed = mSpeed;
            target = setpoint;
        }

        public void setTarget(double setpoint) {
            target = setpoint;
        }

        public void setOffset(double value) {
            offset = value;
        }

        public double pidGet() {
            double speed = MathUtil.clamp(super.calculate(offset, target), -maxSpeed, maxSpeed);
            return speed;
        }

    }
}