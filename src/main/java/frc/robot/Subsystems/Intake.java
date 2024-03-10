package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Subsystems.swerve.SwerveConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Intake extends SubsystemBase implements Loggable {
    private final TalonFX m_intakeMotor; //TODO change to TalonFX
    private double speed = 0;
    private boolean haveGamePiece = false;
    private final Debouncer m_debouncer = new Debouncer(0.1, DebounceType.kRising);
    private static Intake instance;
    
    TalonFXConfiguration intakeTalonConfig = new TalonFXConfiguration();

    private Intake() {
        m_intakeMotor = new TalonFX(IntakeConstants.INTAKE_ID);
        m_intakeMotor.getConfigurator().refresh(intakeTalonConfig);
        final Slot0Configs intakeMotorGains = new Slot0Configs();
        
        intakeMotorGains.kP = SwerveConstants.kP;
        intakeMotorGains.kI = 0;
        intakeMotorGains.kD = 0;
        intakeMotorGains.kS = SwerveConstants.kS;
        intakeMotorGains.kV = SwerveConstants.kV;

        intakeTalonConfig.Slot0 = intakeMotorGains;
        intakeTalonConfig.MotorOutput.Inverted = IntakeConstants.MOTOR_INVERTED;
        intakeTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_intakeMotor.getConfigurator().apply(intakeTalonConfig);
    }

    public static Intake getInstance() {
        if(instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public void onEnable() {
        m_intakeMotor.getConfigurator().refresh(intakeTalonConfig);
        intakeTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_intakeMotor.getConfigurator().apply(intakeTalonConfig);
    }

    public void onDisable() {
        m_intakeMotor.getConfigurator().refresh(intakeTalonConfig);
        intakeTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_intakeMotor.getConfigurator().apply(intakeTalonConfig);
    }

    @Override
    public void periodic() {
        m_intakeMotor.set(speed);
    }

    public void setIntake(double speed) { //TODO can probably hardcode to IntakeConstants.INTAKE_SPEED
        this.speed = speed;
    }

    public void stopIntake() {
        speed = 0;
    }

    public void unjamIntake() {
        speed = -IntakeConstants.MAX_SPEED;
    }

    @Log
    public double getMotorCurrent() {
        return m_intakeMotor.getStatorCurrent().getValueAsDouble();
    }

    @Log
    public double getIntakePosition() {
        return m_intakeMotor.getPosition().getValueAsDouble();
    }

    public void resetIntakePosition() {
        //m_intakeMotor.setPosition(0);
    }

    public void setIntakePosition(double position) {
        //m_intakeMotor.set;
    }

    @Log
    public boolean haveGamePiece() {
        return haveGamePiece;
    }

    public Command intakeUntilTime() { //TODO might just wait for bool change in shooter or button trigger
        return Commands.waitUntil(() -> getMotorCurrent() >= 0.5) //TODO find needed current and movo to Constants
            .andThen(Commands.waitSeconds(3)) //TODO might have wrong notation, check timing
            .andThen(Commands.runOnce(() -> haveGamePiece = true)); 
    }

    public Command intakeUntilDistance() { //TODO might just wait for bool change in Shooter or button trigger
        return Commands.waitUntil(() -> getMotorCurrent() >= 0.5) //TODO find needed current and move to Constants
            .andThen(Commands.waitUntil(() -> getIntakePosition() >= 100) //TODO adjust steps, move to Constants
            .andThen(Commands.runOnce(() -> haveGamePiece = true))); 
    }
    //TODO maybe move the have gamepiece to shooter and measure current + timing there
    
}