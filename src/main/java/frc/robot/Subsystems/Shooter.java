package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import io.github.oblarg.oblog.annotations.Config;

public class Shooter extends SubsystemBase implements Loggable{
    private final CANSparkMax m_rotationMotor;
    private final RelativeEncoder m_rotationEncoder;
    private final SparkPIDController m_rotationPID;
    private final WPI_TalonSRX m_shortMotor;
    //private final WPI_TalonSRX m_longMotor;
    
    private static Shooter instance;

    public double rotationReference = ShooterConstants.INITIALIZED_ANGLE;
    public double longSpeed = 0;
    public double shortSpeed = 0;
    public ShooterDirection shooterDirection = ShooterDirection.FORWARD;

    private Shooter() {
        m_rotationMotor = new CANSparkMax(ShooterConstants.ROTATION_ID, MotorType.kBrushless);
        m_rotationMotor.restoreFactoryDefaults();
        m_rotationMotor.setIdleMode(IdleMode.kBrake);
        m_rotationMotor.setInverted(ShooterConstants.ROTATION_INVERTED);
        m_rotationMotor.setSoftLimit(SoftLimitDirection.kForward, ShooterConstants.ROTATION_LIMIT_FWD);
        m_rotationMotor.setSoftLimit(SoftLimitDirection.kReverse, ShooterConstants.ROTATION_LIMIT_REV);
        m_rotationMotor.setSmartCurrentLimit(ShooterConstants.ROTATION_CURRENT_LIMIT);

        m_rotationEncoder = m_rotationMotor.getEncoder();
        //m_rotationEncoder.setInverted(ShooterConstants.ROTATION_INVERTED);
        m_rotationEncoder.setPositionConversionFactor(ShooterConstants.ROTATION_POSITION_FACTOR);
        m_rotationEncoder.setVelocityConversionFactor(ShooterConstants.ROTATION_VELOCITY_FACTOR);

        m_rotationPID = m_rotationMotor.getPIDController();
        m_rotationPID.setFeedbackDevice(m_rotationEncoder);
        m_rotationPID.setP(ShooterConstants.ROTATION_KP);
        m_rotationPID.setI(ShooterConstants.ROTATION_KI);
        m_rotationPID.setD(ShooterConstants.ROTATION_KD);
        m_rotationPID.setOutputRange(-0.8, 0.8); //TODO probably less

        m_rotationMotor.burnFlash(); //TODO copy setup into swerve

        /*
        m_longMotor = new WPI_TalonSRX(ShooterConstants.LONG_ID);
        m_longMotor.configFactoryDefault();
        m_longMotor.setInverted(ShooterConstants.LONG_INVERTED);
        m_longMotor.configPeakCurrentLimit(ShooterConstants.LONG_CURRENT_LIMIT);
        m_longMotor.configOpenloopRamp(0.2);
        m_longMotor.setNeutralMode(NeutralMode.Coast);
*/
        m_shortMotor = new WPI_TalonSRX(ShooterConstants.SHORT_ID);
        m_shortMotor.configFactoryDefault();
        m_shortMotor.setInverted(ShooterConstants.SHORT_INVERTED);
        m_shortMotor.configPeakCurrentLimit(ShooterConstants.SHORT_CURRENT_LIMIT);
        m_shortMotor.configOpenloopRamp(0.2);
        m_shortMotor.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public void periodic() {
        m_rotationPID.setReference(rotationReference, ControlType.kPosition);
        
        m_shortMotor.set(shortSpeed);
        //m_longMotor.set(longSpeed);
    }

    public void setAngle(double angle) {
        setRotationReference(angle);
    }

    @Config.NumberSlider(defaultValue = ShooterConstants.INITIALIZED_ANGLE, max = ShooterConstants.ROTATION_LIMIT_FWD, min = -ShooterConstants.ROTATION_LIMIT_REV, blockIncrement = 1)
    public void setRotationReference(double input) {
        this.rotationReference = input;
    }

    @Log
    public double getRotationPosition() {
        return m_rotationEncoder.getPosition();
    }

    public double getRotationCurrent() {
        return m_rotationMotor.getOutputCurrent();
    }

    public boolean withinError() {
        return Math.abs(rotationReference - getRotationPosition()) < ShooterConstants.ERROR;
    }

    public void setLongSpeed(double speed) {
        switch (shooterDirection) {
            case FORWARD:
                longSpeed = speed;
                break;
        
            case REVERSE:
                longSpeed = -speed;
                break;
        }
    }

    public void disableLong() {
        longSpeed = 0;
    }

    public void setShortSpeed(double speed) {
        switch (shooterDirection) {
            case FORWARD:
                shortSpeed = speed;
                break;
            case REVERSE:
                shortSpeed = -speed;
                break;
        }
    }

    public void disableShort() {
        shortSpeed = 0;
    }

    public static Shooter getInstance() {
        if(instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    public enum RotationPosition {
        STARTUP, HOME, CHUTE, PODIUM_REVERSE, AMP, PODIUM_OUT; //TODO sketch out all positions
    }

    public enum ShooterDirection { //TODO base the direction on the RotationPosition ENUM - PODIUM_OUT, CHUTE = reverse, other = forward
        FORWARD, REVERSE; //TODO make this more clear - define forward and reverse for both motors to make more sense (eg both forward = both in the same direction, not out)
    }
}
