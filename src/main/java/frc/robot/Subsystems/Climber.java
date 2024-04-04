package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Climber extends SubsystemBase implements Loggable{
    
    CANSparkMax m_leftMotor = new CANSparkMax(ClimberConstants.leftMotorID, MotorType.kBrushless);
    CANSparkMax m_rightMotor = new CANSparkMax(ClimberConstants.rightMotorID, MotorType.kBrushless);

    RelativeEncoder m_leftEncoder;
    RelativeEncoder m_rightEncoder;

    SparkPIDController m_leftPID;
    SparkPIDController m_rightPID;

    private static Climber instance;

    static double leftSpeed = 0;
    static double rightSpeed = 0;

    ClimberState state = ClimberState.IDLE;

    double leftOut = 235;
    double rightOut = 235;

    public static Climber getInstance() {
        if(instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    public Climber() {
        m_leftMotor.restoreFactoryDefaults();
        m_leftMotor.setIdleMode(IdleMode.kBrake);
        m_leftMotor.setInverted(ClimberConstants.leftMotorInverted);
        m_leftMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
        m_leftMotor.setSmartCurrentLimit(70);

        m_leftEncoder = m_leftMotor.getEncoder();
        m_leftPID = m_leftMotor.getPIDController();
        m_leftPID.setFeedbackDevice(m_leftEncoder);
        m_leftPID.setP(0.1);
        m_leftPID.setI(0);
        m_leftPID.setD(0);
        m_leftPID.setOutputRange(-0.85, 0.85);

        m_leftMotor.burnFlash();
        m_leftEncoder.setPosition(0);

        m_rightMotor.restoreFactoryDefaults();
        m_rightMotor.setIdleMode(IdleMode.kBrake);
        m_rightMotor.setInverted(ClimberConstants.rightMotorInverted);
        m_rightMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
        m_rightMotor.setSmartCurrentLimit(70);

        m_rightEncoder = m_rightMotor.getEncoder();
        m_rightPID = m_rightMotor.getPIDController();
        m_rightPID.setFeedbackDevice(m_rightEncoder);
        m_rightPID.setP(0.1); //TODO might be bit higher after the gearbox change
        m_rightPID.setI(0);
        m_rightPID.setD(0);
        m_rightPID.setOutputRange(-1, 1); //TODO check if this is ok, but the speed has to be as high as possible

        m_rightMotor.burnFlash();

        m_rightEncoder.setPosition(0);
    }

    public void init() {
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        autoExtension();
    }

    public Command retrieveLeft() {
        return Commands.runEnd(
        () -> {
            state = ClimberState.CLIMBING;
            m_leftPID.setReference(-0.5, ControlType.kDutyCycle);},
        () -> {
            state = ClimberState.IDLE;
            m_leftPID.setReference(0, ControlType.kDutyCycle);}
        );
    }

    public Command retrieveRight() {
        return Commands.runEnd(
        () -> {
            state = ClimberState.CLIMBING;
            m_rightPID.setReference(-0.5, ControlType.kDutyCycle);},
        () -> {
            state = ClimberState.IDLE;
            m_rightPID.setReference(0, ControlType.kDutyCycle);}
        );
    }

    public Command setState(String stateName) {
        return Commands.runOnce(() -> {
            switch (stateName) {
            case "OUT":
                state = ClimberState.OUT;
                break;
            case "AUTO_OUT":
                if(state.equals(ClimberState.CLIMBING)) { //makes sure that the arms don't extend if we already started climbing

                }else {
                    state = ClimberState.OUT;
                } 
                break;
            case "CLIMBING":
                state = ClimberState.CLIMBING;
                break;
            case "IDLE":
                state = ClimberState.IDLE;
                break;
        }
        });
    }

    public void autoExtension() {
        switch (state) {
            case IDLE:
                break;
            case OUT:
                m_leftPID.setReference(leftOut, ControlType.kPosition);
                m_rightPID.setReference(rightOut, ControlType.kPosition);
                break;
            case CLIMBING:
                break;
        }
    }

    public Command climbLeft() { //TODO maybe create a function that allows us to reset the climber in pit
        return Commands.run(() -> { //TODO test if this runs all the time if called once
            switch (state) {
            case IDLE:
                break;
            case OUT:
                break;
            case CLIMBING:
                m_leftPID.setReference(-0.9, ControlType.kPosition); //TODO keep position or we will break something again
                break;
            }
        });
    }

    public Command climbRight() {
        return Commands.run(() -> {
            switch (state) {
            case IDLE:
                break;
            case OUT:
                break;
            case CLIMBING:
                m_rightPID.setReference(-0.9, ControlType.kPosition);
                break;
            }
        });
    }

    public Command stopLeftClimber() {
        return Commands.runOnce(() -> {
            m_leftPID.setReference(0, ControlType.kDutyCycle);
        });
    }

    public Command stopRightClimber() {
        return Commands.runOnce(() -> {
            m_rightPID.setReference(0, ControlType.kDutyCycle);
        });
    }

    @Log
    public double getLeftEncoder() {
        return m_leftEncoder.getPosition();
    }

    @Log
    public double getRightEncoder() {
        return m_rightEncoder.getPosition();
    }

    @Log
    public String getClimberStateName() {
        return state.toString();
    }

    @Log
    public boolean getLeftArmExtended() {
        return Math.abs(leftOut - getLeftEncoder()) <= 2; //TODO test if this tolerance is close enough
    }

    @Log
    public boolean getRightArmExtended() {
        return Math.abs(rightOut - getRightEncoder()) <= 2; //TODO test if this tolerance is close enough
    }

    public enum ClimberState {
        IDLE, OUT, CLIMBING;
    }
}
