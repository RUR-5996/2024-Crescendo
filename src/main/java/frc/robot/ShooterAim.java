package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterAim {

    static Counter encoder = new Counter(new DigitalInput(0));

    static double CLOCKWISE_TICKS = 0;

    static boolean encoderInverted = false;
    static final double ticksPerDegree = 4096 / 360;

    public static void intakeInit() {
        RobotMap.shooterAim.configFactoryDefault();
        RobotMap.shooterAim.setInverted(false); //use constant
        RobotMap.shooterAim.configOpenloopRamp(0.2);
        RobotMap.shooterAim.configClosedloopRamp(0.2);
        RobotMap.shooterAim.config_kP(0, 0.5);
        RobotMap.shooterAim.config_kI(0, 0);
        RobotMap.shooterAim.config_kD(0, 0);
        RobotMap.shooterAim.setInverted(true);

        RobotMap.shooterAim.set(VictorSPXControlMode.Position, 0);
    }

    public static void periodic() {
        Aim();
    }

    public static void robotPeriodic() {
        getEncoderData();
    }

    public static void Aim() {
        if(RobotMap.controller.getLeftTriggerAxis() > 0.05) {
            RobotMap.shooterAim.set(VictorSPXControlMode.PercentOutput, 0.2);
        } else if (RobotMap.controller.getLeftTriggerAxis() > 0.05) {
            RobotMap.shooterAim.set(VictorSPXControlMode.PercentOutput, -0.2);
        } else {
            RobotMap.shooterAim.set(VictorSPXControlMode.PercentOutput, 0);
        }
    }

    public static void getEncoderData() {
        if(!encoderInverted) {
            CLOCKWISE_TICKS += encoder.get();
        } else {
            CLOCKWISE_TICKS -= encoder.get();
        }
    }

    public static void autoAimShooterInDegrees(double input) {
        RobotMap.shooterAim.set(VictorSPXControlMode.Position, input*ticksPerDegree);
    }
}