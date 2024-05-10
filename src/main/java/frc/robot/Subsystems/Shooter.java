package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Shooter extends SubsystemBase implements Loggable {

    static SwerveDrive SWERVE;

    WPI_TalonSRX m_shortMotor = new WPI_TalonSRX(ShooterConstants.SHORT_ID);
    TalonFX m_longMotor = new TalonFX(ShooterConstants.LONG_ID);
    CANSparkMax m_rotationMotor = new CANSparkMax(6, MotorType.kBrushless);
    RelativeEncoder m_rotationEncoder;
    SparkPIDController m_rotationPID;

    AnalogEncoder m_encoder = new AnalogEncoder(new AnalogInput(0));

    private TalonFXConfiguration shooterTalonConfig = new TalonFXConfiguration();

    static double shortSpeed = 0;
    static double longSpeed = 0;
    static double shooterAngle = 0;
    public ShooterState state = ShooterState.HOME;
    Timer robotTimer;
    double lastTime;
    double coeff = 1;
    boolean longDist;
/*
    public static Shooter getInstance() {
        if(instance == null) {
            instance = new Shooter();
        }
        return instance;
    }
*/
    public Shooter() {

        robotTimer = new Timer();
        robotTimer.reset();

        m_rotationMotor.restoreFactoryDefaults();
        m_rotationMotor.setIdleMode(IdleMode.kBrake);
        m_rotationMotor.setInverted(ShooterConstants.ROTATION_INVERTED);
        m_rotationMotor.setSoftLimit(SoftLimitDirection.kReverse, ShooterConstants.ROTATION_LIMIT_FWD);
        m_rotationMotor.setSoftLimit(SoftLimitDirection.kForward, ShooterConstants.ROTATION_LIMIT_REV);
        m_rotationMotor.setSmartCurrentLimit(ShooterConstants.ROTATION_CURRENT_LIMIT);

        m_rotationEncoder = m_rotationMotor.getEncoder();
        m_rotationEncoder.setPositionConversionFactor(ShooterConstants.ROTATION_POSITION_FACTOR);
        m_rotationEncoder.setVelocityConversionFactor(ShooterConstants.ROTATION_VELOCITY_FACTOR);

        m_rotationPID = m_rotationMotor.getPIDController();
        m_rotationPID.setFeedbackDevice(m_rotationEncoder);
        m_rotationPID.setP(ShooterConstants.ROTATION_KP);
        m_rotationPID.setI(ShooterConstants.ROTATION_KI);
        m_rotationPID.setD(ShooterConstants.ROTATION_KD);
        m_rotationPID.setOutputRange(-0.3, 0.3);

        m_rotationMotor.burnFlash();

        m_shortMotor = new WPI_TalonSRX(ShooterConstants.SHORT_ID);
        m_shortMotor.configFactoryDefault();
        m_shortMotor.setInverted(ShooterConstants.SHORT_INVERTED);
        m_shortMotor.configPeakCurrentLimit(ShooterConstants.SHORT_CURRENT_LIMIT);
        m_shortMotor.configOpenloopRamp(0.2);
        m_shortMotor.setNeutralMode(NeutralMode.Coast);

        m_longMotor.getConfigurator().refresh(shooterTalonConfig);
        shooterTalonConfig.MotorOutput.Inverted = ShooterConstants.LONG_INVERTED;
        shooterTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        ClosedLoopRampsConfigs closedLoopConfig = new ClosedLoopRampsConfigs();
        closedLoopConfig.VoltageClosedLoopRampPeriod = 0.3;
        closedLoopConfig.TorqueClosedLoopRampPeriod = 0.3;
        closedLoopConfig.DutyCycleClosedLoopRampPeriod = 0.3;
        OpenLoopRampsConfigs openLoopConfig = new OpenLoopRampsConfigs();
        openLoopConfig.TorqueOpenLoopRampPeriod = 0.3;
        openLoopConfig.DutyCycleOpenLoopRampPeriod = 0.3;
        openLoopConfig.VoltageOpenLoopRampPeriod = 0.3;
        shooterTalonConfig.OpenLoopRamps = openLoopConfig;
        shooterTalonConfig.ClosedLoopRamps = closedLoopConfig;
        m_longMotor.getConfigurator().apply(shooterTalonConfig);
        lastTime = robotTimer.get();

        
    }

    public void init() {
        m_rotationEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        m_rotationPID.setReference(shooterAngle, ControlType.kPosition);
        m_shortMotor.set(shortSpeed);
        m_longMotor.set(longSpeed);
    }

    public void setNeoEncoder(double position) {
        m_rotationEncoder.setPosition(position);
    }

    public void resetEncoder() {
        m_rotationEncoder.setPosition(0);
    }

    public void setShortSpeed(double speed) {
        shortSpeed = speed;
    }

    public void setLongSpeed(double speed) {
        longSpeed = speed;
    }

    public void setShooterAngle(double angle) {
        shooterAngle = angle;
    }

    public Command setState(String stateName, DoubleSupplier robotAngleDegs) {
        return Commands.runOnce(() -> {
            switch (stateName) {
                case "HOME":
                    state = ShooterState.HOME;
                    setShooterAngle(0);
                    break;
                case "INTAKE":
                    state = ShooterState.INTAKE;
                    setShooterAngle(17);
                    break;
                case "LOADING_STATION":
                    state = ShooterState.LOADING_STATION;
                    setShooterAngle(40);
                    break;
                case "AMP":
                    state = ShooterState.AMP;
                    setShooterAngle(123);
                    break;
                case "SPEAKER":
                    longDist = false;
                    if(robotAngleDegs.getAsDouble() >= -90 && robotAngleDegs.getAsDouble() <= 90) {
                        state = ShooterState.SPEAKER_BACK;
                        setShooterAngle(207); //210
                    } else {
                        state = ShooterState.SPEAKER_FRONT;
                        setShooterAngle(133.5); //130
                    }
                    break;
                case "CLIMBER":
                    state = ShooterState.CLIMBER;
                    setShooterAngle(0); //TODO test ideal angle
                    break;
                case "DEFENSE":
                    state = ShooterState.DEFENSE;
                    setShooterAngle(180);
                    break;
            }
        });
    }

    public Supplier<String> supplyShooterState() {
        Supplier<String> supplier = () -> state.toString();
        return supplier;
    }

    public Command tilt(boolean up) {
        return Commands.runOnce(() -> {
            if(up) {
                setNeoEncoder(getActualPosition() - 1);
            } else {
                setNeoEncoder(getActualPosition() + 1);
            }
        });
    }

    public Command longDistanceTilt() {
        return Commands.runOnce(
            () -> {
                if(state == ShooterState.SPEAKER_FRONT) {
                    longDist = true;
                }
            }
        );
    }

    public Command intake() {
        return Commands.runEnd(()-> {
            switch (state) {
                case HOME:
                    setShortSpeed(0);
                    setLongSpeed(0);
                    break;
                case INTAKE:
                    setShortSpeed(0);
                    setLongSpeed(-0.7);
                    break;
                case LOADING_STATION:
                    setShortSpeed(-0.7);
                    setLongSpeed(0);
                    break;
                case AMP:
                    setShortSpeed(0);
                    setLongSpeed(0);
                    break;
                case SPEAKER_FRONT:
                    setShortSpeed(0);
                    setLongSpeed(0);
                    break;
                case SPEAKER_BACK:
                    setShortSpeed(0);
                    setLongSpeed(0);
                    break;
                case CLIMBER:
                    setShortSpeed(-0.7);
                    setLongSpeed(0);
                    break;
                case DEFENSE:
                    setShortSpeed(0);
                    setLongSpeed(0);
                    break;
            }
        },
        () -> stopShooter());
    }

    public Command deploy() {
        return Commands.runEnd(() -> {
            switch(state) {
                case HOME:
                    setShortSpeed(0);
                    setLongSpeed(0);
                    break;
                case INTAKE:
                    setShortSpeed(0);
                    setLongSpeed(0);
                    break;
                case LOADING_STATION:
                    setShortSpeed(0);
                    setLongSpeed(0);
                    break;
                case AMP:
                    setShortSpeed(0.75);
                    setLongSpeed(-0.5);
                    break;
                case SPEAKER_FRONT:
                    setShortSpeed(-0.75);
                    setLongSpeed(0.95);
                    break;
                case SPEAKER_BACK:
                    setShortSpeed(-0.75);
                    setLongSpeed(0.95);
                    break;
                case CLIMBER:
                    setShortSpeed(-0.75);
                    setLongSpeed(0.80);
                    break;
                case DEFENSE:
                    setShortSpeed(0);
                    setLongSpeed(0);
                    break;
            } 
        }, 
        () -> stopShooter());
    }

    public Command stopShooterCommand() {
        return Commands.runOnce(() -> {
            setShortSpeed(0);
            setLongSpeed(0);
        });
    }

    public void stopShooter() {
        setShortSpeed(0);
        setLongSpeed(0);
    }

    public Command preloadPiece() {
        return Commands.runEnd(() -> {
            setShortSpeed(0.5);
            setLongSpeed(0);
        },
        () -> stopShooter());
    }

    @Log
    public double getActualPosition() {
        return m_rotationEncoder.getPosition();
    }

    @Log
    public double getAbsolutePosition() {
        double value = m_encoder.getAbsolutePosition() * 360 + 360 - ShooterConstants.ROTATION_OFFSET;
        if(value > 360) {
            return value-360;
        } else {
            return value;
        }
    }

    @Log
    public double getShortSpeed() {
        return shortSpeed;
    }

    @Log
    public double getLongSpeed() {
        return longSpeed;
    }

    @Log
    public double getShooterAngle() {
        return shooterAngle;
    }

    public ShooterState getShooterState() {
        return state;
    }

    /*public Command updateSpeakerAngle(DoubleSupplier angleSupplier, Shooter shooter) {
        return Commands.run(
            () -> {
                if(state == ShooterState.SPEAKER_BACK || state == ShooterState.SPEAKER_FRONT){
                    double angle = angleSupplier.getAsDouble();
                    if(angle <= 40 && angle >= -40) {
                        setShooterAngle(205);
                    } 
                    else if((angle < -40 && angle >= -135)||(angle > 40 && angle <= 135)) {
                        setShooterAngle(205);
                    } else if(angle < -135 || angle > 135) {
                        if(longDist) {
                        state = ShooterState.SPEAKER_FRONT;
                        setShooterAngle(126.5); //130
                        } else {
                            setShooterAngle(137);
                        }  
                    }
                }
            }, shooter
        );
    }*/
    
    public Command updateSpeakerAngle(DoubleSupplier angleSupplier, Shooter shooter) { //TODO test
        return Commands.run(
            () -> {
                if(state == ShooterState.SPEAKER_BACK || state == ShooterState.SPEAKER_FRONT){
                    double angle = angleSupplier.getAsDouble();
                    if(angle <= 90 && angle >= -90) {
                        state = ShooterState.SPEAKER_BACK;
                        setShooterAngle(205);
                    } else if(angle < -90 || angle > 90) {
                        state = ShooterState.SPEAKER_FRONT;
                        if(longDist) {
                            setShooterAngle(126.5);
                        } else {
                            setShooterAngle(137);
                        }  
                    }
                }
            }, shooter);
    }

    @Log
    public String getShooterStateName() {
        switch(state) {
                case HOME:
                    return "HOME";
                case INTAKE:
                    return "INTAKE";
                case LOADING_STATION:
                    return "LOADING_STATION";
                case AMP:
                    return "AMP";
                case SPEAKER_FRONT:
                    return "SPEAKER_FRONT";
                case SPEAKER_BACK:
                    return "SPEAKER_BACK";
                case CLIMBER:
                    return "TRAP";
                case DEFENSE:
                    return "DEFENSE";
            } 
        return "NULL";
    }

    @Log
    public double getShortCurrent() {
        return m_shortMotor.getStatorCurrent(); //TODO test supply current
    }

    @Deprecated
    public BooleanSupplier isLoaded() { 
        BooleanSupplier supplier = () -> getShortCurrent() >= 15; //TODO test the current values
        return supplier;
    }

    public enum ShooterState {
        HOME, INTAKE, LOADING_STATION, AMP, SPEAKER_FRONT, SPEAKER_BACK, CLIMBER, DEFENSE;
    }
}