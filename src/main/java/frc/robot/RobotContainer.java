// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Subsystems.SwerveDrive;
import frc.robot.Subsystems.swerve.SwerveConstants;
import frc.robot.util.SendableChooserEnum;
import io.github.oblarg.oblog.Logger;

import java.io.File;
import java.sql.DriverAction;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class RobotContainer {
    private final CommandXboxController xBox = new CommandXboxController(0);

    private AutoBuilder autoBuilder = new AutoBuilder();
    private final AutoBuilder otfBuilder = new AutoBuilder();
    private HashMap<String, Command> eventMap = new HashMap<>();

    private final SendableChooser<PathPlannerPath> autoSelect = new SendableChooser<>();

    private final PowerDistribution pdp = new PowerDistribution(1, ModuleType.kCTRE);
    private final SwerveDrive s_drive = SwerveDrive.getInstance();


  public RobotContainer() {

    Preferences.initBoolean("pFieldRelative", DriverConstants.FIELD_RELATIVE);
    Preferences.initBoolean("pAccelInputs", DriverConstants.ACCELERATED_INPUTS);
    Preferences.initDouble("pDriveGovernor", DriverConstants.DRIVE_GOVERNOR);
    Preferences.initBoolean("pOptimizeSteering", SwerveConstants.OPTIMIZESTEERING);
    Preferences.initDouble("pKPRotationController", SwerveConstants.P_ROTATION_CONTROLLER);
    Preferences.initDouble("pKIRotationController", SwerveConstants.D_ROTATION_CONTROLLER);
    Preferences.initDouble("pKDRotationController", SwerveConstants.I_ROTATION_CONTROLLER);

    Shuffleboard.getTab("pdp").add("PDP", pdp).withWidget(BuiltInWidgets.kPowerDistribution);

    loadPaths();
    configureBindings();

    Shuffleboard.getTab("selectors")
      .add("Auto Chooser", autoSelect)
      .withWidget(BuiltInWidgets.kComboBoxChooser)
      .withPosition(17, 0)
      .withSize(3, 1);

    /*otfBuilder.configureHolonomic(
      s_drive::getOdometryPose, //poseSupplier
      s_drive::resetOdometry, //resetPose
      s_drive::getChassisSpeeds, //robot relative speed supplier
      s_drive::setAutoChassisSpeeds, //robot relative output


      s_drive);*/

    s_drive.setDefaultCommand(
      s_drive.joystickDriveCommand(
        () -> xBox.getLeftY(),
        () -> xBox.getLeftX(),
        xBox::getRightX)
      .withName("DefaultDrive"));

    Logger.configureLoggingAndConfig(this, false);

  }

  private void configureBindings() {

    //disable sequence
    new Trigger(DriverStation::isDisabled)
      .onTrue(Commands.waitSeconds(5)
        .andThen(Commands.repeatingSequence(Commands.runOnce(s_drive::setToCoast)).ignoringDisable(true)
          .withName("setToCoast")));
        
    //enable sequence
    new Trigger(DriverStation::isEnabled)
      .onTrue(Commands.runOnce(s_drive::setToBrake).ignoringDisable(true)
      .withName("setToBreak"));

  }

  private void loadPaths() {
    autoBuilder.configureHolonomic( //TODO fix this shit
      s_drive::getOdometryPose,
      s_drive::resetOdometry,
      s_drive::getChassisSpeeds,
      s_drive::setAutoChassisSpeeds,
      AutoConstants.autoConfig,
      () -> {
        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent()){return alliance.get() == DriverStation.Alliance.Red;}
        else {return false;}
      },
      s_drive);

    HashMap<String, PathConstraints> constraintsOverride = new HashMap<>();

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
  }

  public Command getAutonomousCommand() {
    return autoBuilder.followPath(autoSelect.getSelected());
  }

  public enum PickupLocation {
    GROUND, CHUTE
  }

}
