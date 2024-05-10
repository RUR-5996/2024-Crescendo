// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LEDs;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SwerveDrive;
import io.github.oblarg.oblog.Loggable;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

public class RobotContainer implements Loggable {
    private final CommandXboxController xBox = new CommandXboxController(0);
    private final GenericHID coDrive = new GenericHID(1); //TODO test if this registeres correctly
    private final Trigger b1 = new JoystickButton(coDrive, 1);
    private final Trigger b2 = new JoystickButton(coDrive, 2);
    private final Trigger b3 = new JoystickButton(coDrive, 3);
    private final Trigger b4 = new JoystickButton(coDrive, 4);
    private final Trigger b5 = new JoystickButton(coDrive, 5);
    private final Trigger b6 = new JoystickButton(coDrive, 6);
    private final Trigger b7 = new JoystickButton(coDrive, 7);
    private final Trigger b8 = new JoystickButton(coDrive, 8);
    private final Trigger b9 = new JoystickButton(coDrive, 9);
    private final Trigger b10 = new JoystickButton(coDrive, 10);
    private final Trigger b11 = new JoystickButton(coDrive, 11);

    private final CommandXboxController pitBox = new CommandXboxController(2);

    private final SendableChooser<PathPlannerPath> autoSelect = new SendableChooser<>();
    private final SendableChooser<Command> autoChooser;

    private PowerDistribution pdp;
    private SwerveDrive SWERVE;
    public static Shooter SHOOTER;
    private static Intake INTAKE;
    public static Climber CLIMBER;
    private LEDs LEDS;

    static String shooterState = "INTAKE";
    static String climberState = "IDLE";
    static String intakeState = "IDLE";


  public RobotContainer() {

    pdp = new PowerDistribution(0, ModuleType.kCTRE);
    SWERVE = new SwerveDrive();
    SHOOTER = new Shooter();
    INTAKE = new Intake();
    CLIMBER = new Climber();
    LEDS = new LEDs();

    Shuffleboard.getTab("pdp").add("PDP", pdp).withWidget(BuiltInWidgets.kPowerDistribution);

    loadPaths();
    configureBindings();

    Shuffleboard.getTab("selectors")
      .add("Auto Chooser", autoSelect)
      .withWidget(BuiltInWidgets.kComboBoxChooser)
      .withPosition(17, 0)
      .withSize(3, 1);

    SWERVE.setDefaultCommand(SWERVE.joystickDrive(xBox::getLeftX, xBox::getLeftY, xBox::getRightX, SWERVE));
    SHOOTER.setDefaultCommand(SHOOTER.updateSpeakerAngle(SWERVE.supplyRobotAngleDegrees(), SHOOTER));

    NamedCommands.registerCommand("g_intake", SHOOTER.setState("INTAKE", SWERVE.supplyRobotAngleDegrees()).andThen(Commands.parallel(INTAKE.intake(), SHOOTER.intake())).withTimeout(1.5));
    NamedCommands.registerCommand("set_amp", SHOOTER.setState("AMP", SWERVE.supplyRobotAngleDegrees()));
    NamedCommands.registerCommand("set_speaker", SHOOTER.setState("SPEAKER", SWERVE.supplyRobotAngleDegrees()).andThen(Commands.waitSeconds(0.5)).andThen(SHOOTER.preloadPiece().withTimeout(0.2)));
    NamedCommands.registerCommand("shoot", SHOOTER.deploy().withTimeout(0.5));
    NamedCommands.registerCommand("set_intake", SHOOTER.setState("INTAKE", SWERVE.supplyRobotAngleDegrees()));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous", autoChooser);
  }
  private void configureBindings() {

    //xBox.a().toggleOnTrue(SWERVE.configState(() -> "NULL")); //disables all assissts
    xBox.b().toggleOnTrue(SWERVE.configState("CLIMBER"));
    xBox.y().toggleOnTrue(SHOOTER.preloadPiece().withTimeout(0.1));
    xBox.x().whileTrue(INTAKE.reverse());
    //xBox.start().toggleOnTrue(SWERVE.configState(""));

    xBox.rightBumper().whileTrue(Commands.parallel(SHOOTER.intake(), INTAKE.intake()));
    xBox.leftBumper().whileTrue(SHOOTER.deploy()); //TODO what happens if both are pressed?

    b2.toggleOnTrue(SHOOTER.setState("INTAKE", SWERVE.supplyRobotAngleDegrees()));
    b2.toggleOnTrue(SWERVE.configState("INTAKE"));
    b3.toggleOnTrue(SHOOTER.setState("LOADING_STATION", SWERVE.supplyRobotAngleDegrees()));
    b3.toggleOnTrue(SWERVE.configState("LOADING_STATION"));
    b4.toggleOnTrue(SHOOTER.setState("AMP", SWERVE.supplyRobotAngleDegrees()));
    b4.toggleOnTrue(SWERVE.configState("AMP"));
    b5.toggleOnTrue(SHOOTER.setState("SPEAKER", SWERVE.supplyRobotAngleDegrees()).andThen(SHOOTER.preloadPiece().withTimeout(0.2)));
    b5.toggleOnTrue(SWERVE.configState("SPEAKER_FRONT"));

    b7.toggleOnTrue(SHOOTER.tilt(false));
    b6.toggleOnTrue(SHOOTER.tilt(true));
    
    //b9.toggleOnTrue(CLIMBER.halveClimber().withTimeout(2).andThen(Commands.parallel(CLIMBER.stopLeftClimber(), CLIMBER.stopRightClimber())));
    b9.toggleOnTrue(SHOOTER.setState("DEFENSE", SWERVE.supplyRobotAngleDegrees()));
    b9.toggleOnTrue(SWERVE.configState("DEFENSE"));


    b1.toggleOnTrue(SHOOTER.longDistanceTilt());
    b10.toggleOnTrue(CLIMBER.setState("OUT"));
    b10.toggleOnTrue(SWERVE.configState("INTAKE"));
    b11.toggleOnTrue(CLIMBER.setState("CLIMBING"));
    b11.toggleOnTrue(SWERVE.configState("CLIMBER"));


    b7.toggleOnTrue(LEDS.resetTimer().andThen(LEDS.signalAmplify().withTimeout(3)));

    //extends arms on 15seconds
    new Trigger(isEndgame()).toggleOnTrue(Commands.parallel(CLIMBER.setState("AUTO_OUT"), LEDS.signalEndgame()));

    xBox.leftTrigger().whileTrue(CLIMBER.climbLeft());
    xBox.rightTrigger().whileTrue(CLIMBER.climbRight());

    b8.toggleOnTrue(SWERVE.resetGyro());

    pitBox.leftTrigger().whileTrue(CLIMBER.retrieveLeft());
    pitBox.rightTrigger().whileTrue(CLIMBER.retrieveRight());
    
  }

  private void loadPaths() {
    AutoBuilder.configureHolonomic(
      SWERVE::getOdometryPose,
      SWERVE::resetOdometry,
      SWERVE::getChassisSpeeds,
      SWERVE::setAutoChassisSpeeds,
      AutoConstants.autoConfig,
      () -> {
        /*if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
          return true;
        } else {
          return false;
        }*/
        return false;
      },
      SWERVE);
  }

  public static void periodic() {
    shooterState = SHOOTER.getShooterStateName();
    climberState = CLIMBER.getClimberStateName();
    intakeState = INTAKE.getIntakeStateName();
  }
 
  public BooleanSupplier isEndgame() {
    BooleanSupplier bool = () -> DriverStation.getMatchTime() <= 15; //TODO test if this works
    return bool;
  }

  public Command getAutoCommand() {
    return autoChooser.getSelected();
  }
}