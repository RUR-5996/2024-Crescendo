package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Subsystems.Shooter.ShooterState;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import frc.robot.Robot;

public class Intake extends SubsystemBase implements Loggable{
    TalonFX m_intakeMotor = new TalonFX(IntakeConstants.motorID);
    private TalonFXConfiguration intakeTalonConfig = new TalonFXConfiguration();
    
    public ShooterState shooterState = Robot.SHOOTER.state;

    private static Intake instance;

    static double speed = 0;
    public IntakeState state = IntakeState.IDLE;

    public static Intake getInstance() {
        if(instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private Intake() {
        m_intakeMotor.getConfigurator().refresh(intakeTalonConfig);
        intakeTalonConfig.MotorOutput.Inverted = IntakeConstants.inverted;
        intakeTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_intakeMotor.getConfigurator().apply(intakeTalonConfig);
    }

    @Override
    public void periodic() {
        m_intakeMotor.set(speed);
    }

    public Command intake(String shooterStateName){
        return Commands.run(() -> {
            switch(shooterStateName) {
                case "HOME":
                    speed = 0;
                    break;
                case "INTAKE":
                    speed = IntakeConstants.motorSpeed;
                    break;
                case "LOADING_STATION":
                    speed = 0;
                    break;
                case "AMP":
                    speed = 0;
                    break;
                case "SPEAKER_FRONT":
                    speed = 0;
                    break;
                case "SPEAKER_BACK":
                    speed = 0;
                    break;
                case "REVERSE":
                    speed = -IntakeConstants.motorSpeed;
                    break;
                case "NULL":
                    speed = 0;
                    break;
            }
        });
    }

    @Log
    public String getIntakeStateName() {
        return state.toString();
    }

    @Log
    public double getIntakeCurrent() {
        return m_intakeMotor.getStatorCurrent().getValueAsDouble();
    }

    public BooleanSupplier isLoaded() {
        BooleanSupplier supplier = () -> getIntakeCurrent() > 15; //TODO check the values
        return supplier;
    }

    public enum IntakeState { //TODO can be unused
        IDLE, INTAKING; //TODO more?
    }
}
