// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
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
import io.github.oblarg.oblog.Logger;

import java.io.File;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
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

    NamedCommands.registerCommand("g_intake", SHOOTER.setState("INTAKE", SWERVE.supplyRobotAngleDegrees()).andThen(Commands.parallel(INTAKE.intake("INTAKE"), SHOOTER.intake())).withTimeout(0.5));
    NamedCommands.registerCommand("set_amp", SHOOTER.setState("AMP", SWERVE.supplyRobotAngleDegrees()));
    NamedCommands.registerCommand("set_speaker", SHOOTER.setState("SPEAKER", SWERVE.supplyRobotAngleDegrees()).andThen(Commands.waitSeconds(0.5)).andThen(SHOOTER.preloadPiece().withTimeout(0.2)));
    NamedCommands.registerCommand("shoot", SHOOTER.deploy().withTimeout(0.5));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous", autoChooser);
  }

  private void configureBindings() {

    //b1.toggleOnTrue(SWERVE.configState(SHOOTER.supplyShooterState())); //TODO probably rebind onto the main controller for the driver to enable autonomous features
    //TODO put one button for rotation controller and one button for tag/note controller (or require double click for tag/note)
    //xBox.a().toggleOnTrue(SWERVE.configState(() -> "NULL")); //disables all assissts
    //xBox.b().toggleOnFalse(SWERVE.toggleSlowMode()); //might get removed, no need to go slow
    xBox.b().toggleOnTrue(SWERVE.configState("CLIMBER"));
    xBox.y().toggleOnTrue(SHOOTER.preloadPiece().withTimeout(0.1));
    xBox.x().whileTrue(INTAKE.intake("REVERSE"));
    //xBox.start().toggleOnTrue(SWERVE.configState(""));

    xBox.rightBumper().whileTrue(Commands.parallel(SHOOTER.intake(), INTAKE.intake(shooterState)).andThen(Commands.parallel(SHOOTER.intake(), INTAKE.intake(shooterState)).withTimeout(0.5).andThen(Commands.parallel(SHOOTER.stopShooterCommand(), INTAKE.stopIntake())))); //TODO interrupt once gamepiece is in the main shooter assembly
    // xBox.rightBumper().whileTrue(INTAKE.intake(shooterState));
    xBox.leftBumper().whileTrue(SHOOTER.deploy()); //TODO what happens if both are pressed?
    //xBox.rightBumper().toggleOnFalse(SHOOTER.preloadPiece().withTimeout(0.75)); //TODO test if it runs or if it just triggers once, also test timing

    b2.toggleOnTrue(SHOOTER.setState("INTAKE", SWERVE.supplyRobotAngleDegrees()));
    b2.toggleOnTrue(SWERVE.configState("INTAKE"));
    b3.toggleOnTrue(SHOOTER.setState("LOADING_STATION", SWERVE.supplyRobotAngleDegrees()));
    b3.toggleOnTrue(SWERVE.configState("LOADING_STATION"));
    b4.toggleOnTrue(SHOOTER.setState("AMP", SWERVE.supplyRobotAngleDegrees()));
    b4.toggleOnTrue(SWERVE.configState("AMP"));
    b5.toggleOnTrue(SHOOTER.setState("SPEAKER", SWERVE.supplyRobotAngleDegrees()).andThen(SHOOTER.preloadPiece().withTimeout(0.2)));
    b5.toggleOnTrue(SWERVE.configState("SPEAKER_FRONT"));

    b10.toggleOnTrue(CLIMBER.setState("OUT"));
    b10.toggleOnTrue(SWERVE.configState("INTAKE"));
    b11.toggleOnTrue(CLIMBER.setState("CLIMBING"));


    b7.toggleOnTrue(LEDS.resetTimer().andThen(LEDS.signalAmplify().withTimeout(3)));

    //extends arms on 15seconds
    new Trigger(isEndgame()).toggleOnTrue(Commands.parallel(CLIMBER.setState("AUTO_OUT"), LEDS.signalEndgame()));
    new Trigger(SHOOTER.isLoaded()).toggleOnTrue(LEDS.signalIntake().withTimeout(3));
    new Trigger(INTAKE.isLoaded()).toggleOnTrue(LEDS.signalIntake().withTimeout(3));

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
        return false;
      },
      SWERVE);
    //HashMap<String, PathConstraints> constraintsOverride = new HashMap<>();

    /*List<File> files = List.of(
      Objects.requireNonNull(new File(Filesystem.getDeployDirectory(), "pathplanner")
        .listFiles((dir, name) -> name.endsWith(".path"))));
        
    for(File file : files) {
      String pathName = file.getName().split("\\.")[0];
      PathConstraints constraints = constraintsOverride.getOrDefault(pathName,
        new PathConstraints(AutoConstants.MAX_VELOCITY, AutoConstants.MAX_ACCELERATION, AutoConstants.MAX_ROT_VELOCITY, AutoConstants.MAX_ROT_ACCELERATION));
      autoSelect.addOption(pathName, PathPlannerPath.fromPathFile(file.getName())); //TODO test if this works with PathPlannerPath or if it has to be PathPlannerTrajectory
    }

    autoSelect.setDefaultOption(files.get(0).getName().split("\\.")[0], PathPlannerPath.fromPathFile(files.get(0).getName()));
    */
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