package frc.robot;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;

public class Shooter {
    static Counter intakeEncoder = new Counter(new DigitalInput(0));

    static final boolean inveted = false;

    public static void intakeInit() {
        RobotMap.shooter.configFactoryDefault();
        RobotMap.shooter.setInverted(false); //use constant
        RobotMap.shooter.configOpenloopRamp(0.2);
        RobotMap.shooter.configClosedloopRamp(0.2);
        RobotMap.shooter.config_kP(0, 0.5);
        RobotMap.shooter.config_kI(0, 0);
        RobotMap.shooter.config_kD(0, 0);
        RobotMap.shooter.setInverted(true);
    }

    public static void periodic() {
        //testMode();
        shoot();
    }

    public static void manualShoot() {
        if(RobotMap.controller.getLeftBumper()) {
            //get it unstuck
            RobotMap.shooter.set(VictorSPXControlMode.PercentOutput, -0.2);
        } else if (RobotMap.controller.getRightBumper()) {
            RobotMap.shooter.set(VictorSPXControlMode.PercentOutput, 0.8);
        } else {
            RobotMap.shooter.set(VictorSPXControlMode.PercentOutput, 0);
        }
    }

    public static void shoot() {
        if(RobotMap.secondController.getLeftBumper()) {
            //get it unstuck
            RobotMap.shooter.set(VictorSPXControlMode.PercentOutput, 0.8);
        } else if (RobotMap.secondController.getRightBumper()) {
            RobotMap.shooter.set(VictorSPXControlMode.PercentOutput, 0.8);
        } else {
            RobotMap.shooter.set(VictorSPXControlMode.PercentOutput, 0);
        }
    }

    public static void testMode() {
        if(RobotMap.secondController.getLeftBumper()) {
            RobotMap.shooter.set(VictorSPXControlMode.PercentOutput, 0.4);
        } else if (RobotMap.secondController.getRightBumper()) {
            RobotMap.shooter.set(VictorSPXControlMode.PercentOutput, -0.4);
        } else {
            RobotMap.shooter.set(VictorSPXControlMode.PercentOutput, 0);
        }
    }
}