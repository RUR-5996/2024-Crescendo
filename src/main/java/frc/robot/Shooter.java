package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ShooterConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Shooter implements Loggable {

    WPI_TalonSRX m_shortMotor = new WPI_TalonSRX(10);
    CANSparkMax m_rotationMotor = new CANSparkMax(6, MotorType.kBrushless);
    RelativeEncoder m_rotationEncoder;
    SparkPIDController m_rotationPID;

    XboxController controller = Robot.controller;
    GenericHID sController = Robot.sController;
    GenericHID tController = Robot.tController;
    JoystickButton b1 = new JoystickButton(sController, 9); //TODO naportovat asi na 2 ovladace
    JoystickButton b2 = new JoystickButton(sController, 2);
    JoystickButton b3 = new JoystickButton(sController, 3);
    JoystickButton b4 = new JoystickButton(sController, 4);
    JoystickButton b5 = new JoystickButton(sController, 5);
    JoystickButton b6 = new JoystickButton(sController, 6);
    JoystickButton b7 = new JoystickButton(sController, 7);
    JoystickButton b8 = new JoystickButton(sController, 8); //TODO nepouzivat na soutez, jen na testing
    
    public static Shooter instance;

    static double shortSpeed = 0;
    static double longSpeed = 0;
    static double shooterAngle = 0;
    public ShooterState state = ShooterState.HOME;

    public static Shooter getInstance() {
        if(instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {
        m_rotationMotor.restoreFactoryDefaults();
        m_rotationMotor.setIdleMode(IdleMode.kBrake);
        m_rotationMotor.setInverted(ShooterConstants.ROTATION_INVERTED);
        m_rotationMotor.setSoftLimit(SoftLimitDirection.kForward, ShooterConstants.ROTATION_LIMIT_FWD);
        m_rotationMotor.setSoftLimit(SoftLimitDirection.kReverse, ShooterConstants.ROTATION_LIMIT_REV);
        m_rotationMotor.setSmartCurrentLimit(ShooterConstants.ROTATION_CURRENT_LIMIT);

        m_rotationEncoder = m_rotationMotor.getEncoder();
        m_rotationEncoder.setPositionConversionFactor(ShooterConstants.ROTATION_POSITION_FACTOR);
        m_rotationEncoder.setVelocityConversionFactor(ShooterConstants.ROTATION_VELOCITY_FACTOR);

        m_rotationPID = m_rotationMotor.getPIDController();
        m_rotationPID.setFeedbackDevice(m_rotationEncoder);
        m_rotationPID.setP(ShooterConstants.ROTATION_KP);
        m_rotationPID.setI(ShooterConstants.ROTATION_KI);
        m_rotationPID.setD(ShooterConstants.ROTATION_KD);
        m_rotationPID.setOutputRange(-0.3, 0.3); //TODO probably less

        m_rotationMotor.burnFlash(); //TODO copy setup into swerve

        m_shortMotor = new WPI_TalonSRX(ShooterConstants.SHORT_ID);
        m_shortMotor.configFactoryDefault();
        m_shortMotor.setInverted(ShooterConstants.SHORT_INVERTED);
        m_shortMotor.configPeakCurrentLimit(ShooterConstants.SHORT_CURRENT_LIMIT);
        m_shortMotor.configOpenloopRamp(0.2);
        m_shortMotor.setNeutralMode(NeutralMode.Coast);

        resetEncoder();
    }

    public void periodic() {

        setState();

        if(controller.getLeftBumper()) { //deploy
            deploy();
        } else if(controller.getRightBumper()) { //intake
            intake();
        } else {
            stopShooter();
        }

        if(b8.getAsBoolean()) { //TODO testing only
            resetEncoder();
        }

        m_rotationPID.setReference(shooterAngle, ControlType.kPosition);
        m_shortMotor.set(shortSpeed);

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

    public void setState() {
        if(b1.getAsBoolean()) {
            state = ShooterState.HOME;
            setShooterAngle(0);
        }
        if(b2.getAsBoolean()) {
            state = ShooterState.INTAKE;
            setShooterAngle(20);
        }
        if(b2.getAsBoolean()) {
            state = ShooterState.LOADING_STATION;
            setShooterAngle(40);
        }
        if(b3.getAsBoolean()) {
            state = ShooterState.SHORT_SPEAKER; //TODO asi nepouzitelny, budeme strilet pres LONG side, dame tam falcon
            setShooterAngle(40);
        }
        if(b4.getAsBoolean()) {
            state = ShooterState.AMP;
            setShooterAngle(115);
        }
        if(b5.getAsBoolean()) {
            state = ShooterState.LONG_SPEAKER_FRONT;
            setShooterAngle(130);
        }
        if(b6.getAsBoolean()) {
            state = ShooterState.LONG_SPEAKER;
            setShooterAngle(230);
        }
    }

    public void intake() {
        switch (state) {
            case HOME:
                setShortSpeed(0);
                setLongSpeed(0);
                break;
            case INTAKE:
                setShortSpeed(0);
                setLongSpeed(-0.9);
                break;
            case LOADING_STATION:
                setShortSpeed(-0.9);
                setLongSpeed(0);
                break;
            case SHORT_SPEAKER:
                setShortSpeed(0);
                setLongSpeed(0);
                break;
            case AMP:
                setShortSpeed(0);
                setLongSpeed(0);
                break;
            case LONG_SPEAKER_FRONT:
                setShortSpeed(0);
                setLongSpeed(0);
                break;
            case LONG_SPEAKER:
                setShortSpeed(0);
                setLongSpeed(0);
                break;
        }
    }

    public void deploy() {
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
            case SHORT_SPEAKER:
                setShortSpeed(0.85);
                setLongSpeed(-0.6);
                break;
            case AMP:
                setShortSpeed(0.75);
                setLongSpeed(-0.5);
                break;
            case LONG_SPEAKER_FRONT:
                setShortSpeed(-0.75);
                setLongSpeed(0.85);
                break;
            case LONG_SPEAKER:
                setShortSpeed(-0.75);
                setLongSpeed(0.85);
                break;
        }
    }

    public void stopShooter() {
        setShortSpeed(0);
        setLongSpeed(0);
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

    @Log
    public String getShooterStateName() {
        return state.toString();
    }

    public enum ShooterState {
        HOME, INTAKE, LOADING_STATION, SHORT_SPEAKER, AMP, LONG_SPEAKER_FRONT, LONG_SPEAKER
    }
}
