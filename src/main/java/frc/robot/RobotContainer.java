// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Subsystems.SwerveDrive;
import frc.robot.Subsystems.LimeLight;
import frc.robot.Subsystems.LEDs;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

public class RobotContainer {

  private final CommandXboxController xBox = new CommandXboxController(0);
  private final SendableChooser<Command> autoChooser;

  private PowerDistribution pdp;
  private SwerveDrive SWERVE;
  private LEDs LEDController;
  
  public RobotContainer() {
    SWERVE = new SwerveDrive();
    pdp = new PowerDistribution(0, ModuleType.kCTRE);
    LEDController = new LEDs();

    loadPaths();

    Shuffleboard.getTab("pdp").add("PDP", pdp).withWidget(BuiltInWidgets.kPowerDistribution);
    SWERVE.setDefaultCommand(SWERVE.joystickDrive(xBox::getLeftX, xBox::getLeftY, xBox::getRightX, SWERVE));
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous", autoChooser);
  }

  private void configureBindings() {
    xBox.b().toggleOnTrue(SWERVE.toggleSlowMode());
    xBox.leftBumper().onTrue(new Command(() -> {
      double[] relativePosition = LimeLight.getRelativePos();
      SmartDashboard.putData("Position", relativePosition);
      LEDController.setColour(((int)relativePosition[3] % 2 == 0) ? Constants.ColourConstants.FLASHBANG : Constants.ColourConstants.PINK);
    }));
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


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}