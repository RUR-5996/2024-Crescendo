package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Intake extends SubsystemBase implements Loggable{
    TalonFX m_intakeMotor = new TalonFX(IntakeConstants.motorID);
    private TalonFXConfiguration intakeTalonConfig = new TalonFXConfiguration();

    private static Intake instance;

    static double speed = 0;
    public IntakeState state = IntakeState.IDLE;

    @Deprecated
    public static Intake getInstance() {
        if(instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public Intake() {
        m_intakeMotor.getConfigurator().refresh(intakeTalonConfig);
        intakeTalonConfig.MotorOutput.Inverted = IntakeConstants.inverted;
        intakeTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_intakeMotor.getConfigurator().apply(intakeTalonConfig);
    }

    @Override
    public void periodic() {
        m_intakeMotor.set(speed);
    }

    public Command intake(){
        return Commands.runEnd(() -> {
            switch(RobotContainer.SHOOTER.state) {
                
                case HOME:
                    speed = 0;
                    break;
                case INTAKE:
                    speed = IntakeConstants.motorSpeed;
                    break;
                case LOADING_STATION:
                    speed = 0;
                    break;
                case AMP:
                    speed = 0;
                    break;
                case SPEAKER_FRONT:
                    speed = 0;
                    break;
                case SPEAKER_BACK:
                    speed = 0;
                    break;
                case CLIMBER:
                    speed = 0;
                    break;
                case DEFENSE:
                    speed = 0;
                    break;
            }
        },
        () -> speed = 0);
    }

    public Command reverse() {
        return Commands.runEnd(
            () -> speed = -IntakeConstants.motorSpeed,
            () -> speed = 0
        );
    }

    public Command stopIntake() {
        return Commands.runOnce(() -> speed = 0);
    }

    @Log
    public String getIntakeStateName() {
        return state.toString();
    }

    @Log
    public double getIntakeCurrent() {
        return m_intakeMotor.getStatorCurrent().getValueAsDouble();
    }

    @Deprecated
    public BooleanSupplier isLoaded() {
        BooleanSupplier supplier = () -> getIntakeCurrent() > 15; //TODO check the values
        return supplier;
    }

    public enum IntakeState { //TODO can be unused
        IDLE, INTAKING; //TODO more?
    }
}
