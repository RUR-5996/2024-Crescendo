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

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

public class RobotContainer implements Loggable {
    private final CommandXboxController xBox = new CommandXboxController(0);
    private final GenericHID coDrive = new GenericHID(1); //TODO change to address axis if needed
    //private final Trigger b1 = new JoystickButton(coDrive, 1); //TODO change to appropriate buttons
    //private final Trigger b2 = new JoystickButton(coDrive, 2);
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
    private final Trigger b12 = new JoystickButton(coDrive, 12);

    private static AutoBuilder autoBuilder = new AutoBuilder();
    private HashMap<String, Command> eventMap = new HashMap<>();

    private final SendableChooser<PathPlannerPath> autoSelect = new SendableChooser<>();

    private final PowerDistribution pdp = new PowerDistribution(1, ModuleType.kCTRE);
    private final SwerveDrive SWERVE = SwerveDrive.getInstance();
    private final Shooter SHOOTER = Shooter.getInstance();
    private final Intake INTAKE = Intake.getInstance();
    private final Climber CLIMBER = Climber.getInstance();
    private final LEDs LEDS = LEDs.getInstance();

    String shooterState = "HOME";
    String climberState = "IDLE";
    String intakeState = "IDLE";


  public RobotContainer() {

    Shuffleboard.getTab("pdp").add("PDP", pdp).withWidget(BuiltInWidgets.kPowerDistribution);

    loadPaths();
    configureBindings();

    Shuffleboard.getTab("selectors")
      .add("Auto Chooser", autoSelect)
      .withWidget(BuiltInWidgets.kComboBoxChooser)
      .withPosition(17, 0)
      .withSize(3, 1);

    SWERVE.setDefaultCommand(
      SWERVE.joystickDrive(xBox::getLeftX, xBox::getLeftY, xBox::getRightX));

    Logger.configureLoggingAndConfig(this, false);

  }

  private void configureBindings() {

    b1.toggleOnTrue(SWERVE.configState(shooterState)); //TODO probably rebind onto the main controller for the driver to enable autonomous features
    //TODO put one button for rotation controller and one button for tag/note controller (or require double click for tag/note)
    xBox.a().toggleOnTrue(SWERVE.configState("NULL")); //disables all assissts
    xBox.b().toggleOnFalse(SWERVE.toggleSlowMode()); //might get removed, no need to go slow

    xBox.x().onTrue(INTAKE.intake("REVERSE"));

    xBox.rightBumper().onTrue(Commands.parallel(SHOOTER.intake(), INTAKE.intake(shooterState)).until(SHOOTER::hasNote)); //TODO interrupt once gamepiece is in the main shooter assembly
    xBox.leftBumper().onTrue(SHOOTER.deploy()); //TODO what happens if both are pressed?
    xBox.rightBumper().toggleOnFalse(SHOOTER.preloadPiece().withTimeout(0.75)); //TODO test if it runs or if it just triggers once, also test timing

    b9.toggleOnTrue(SHOOTER.setState("HOME", SWERVE.getOdometryDegrees()));
    b2.toggleOnTrue(SHOOTER.setState("INTAKE", SWERVE.getOdometryDegrees()));
    b3.toggleOnTrue(SHOOTER.setState("LOADING_STATION", SWERVE.getOdometryDegrees()));
    b4.toggleOnTrue(SHOOTER.setState("AMP", SWERVE.getOdometryDegrees()));
    b5.toggleOnTrue(SHOOTER.setState("SPEAKER", SWERVE.getOdometryDegrees()));
    b6.toggleOnTrue(SHOOTER.setState("TRAP", SWERVE.getOdometryDegrees()));

    b10.toggleOnTrue(CLIMBER.setState("OUT"));
    b11.toggleOnTrue(CLIMBER.setState("CLIMBING"));

    b12.toggleOnTrue(LEDS.resetTimer().andThen(LEDS.signalAmplify().withTimeout(3)));

    //extends arms on 15seconds
    new Trigger(isEndgame()).toggleOnTrue(Commands.parallel(CLIMBER.setState("OUT"), LEDS.signalEndgame()));
    new Trigger(SHOOTER.isLoaded()).toggleOnTrue(LEDS.signalIntake().withTimeout(3));
    new Trigger(INTAKE.isLoaded()).toggleOnTrue(LEDS.signalIntake().withTimeout(3));

    xBox.leftTrigger().onTrue(CLIMBER.climbLeft());
    xBox.rightTrigger().onTrue(CLIMBER.climbRight());
  }

  private void loadPaths() {
    autoBuilder.configureHolonomic( //TODO fix this shit
      SWERVE::getOdometryPose,
      SWERVE::resetOdometry,
      SWERVE::getChassisSpeeds,
      SWERVE::setAutoChassisSpeeds,
      AutoConstants.autoConfig,
      () -> {
        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent()){return alliance.get() == DriverStation.Alliance.Red;}
        else {return false;}
      },
      SWERVE);
    HashMap<String, PathConstraints> constraintsOverride = new HashMap<>();
/*
    List<File> files = List.of(
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

  public void periodic() {
    shooterState = SHOOTER.getShooterStateName();
    climberState = CLIMBER.getClimberStateName();
    intakeState = INTAKE.getIntakeStateName();
  }

  public Command getAutonomousCommand() {
    return autoBuilder.followPath(autoSelect.getSelected());
  }
 
  public BooleanSupplier isEndgame() {
    BooleanSupplier bool = () -> DriverStation.getMatchTime() <= 15; //TODO test if this works
    return bool;
  }

}