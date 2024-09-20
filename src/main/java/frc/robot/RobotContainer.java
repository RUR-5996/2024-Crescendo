// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveDrive;

public class RobotContainer {

  private final CommandXboxController xBox = new CommandXboxController(0);

  private PowerDistribution pdp;
  private SwerveDrive SWERVE;
  
  public RobotContainer() {
    SWERVE = new SwerveDrive();
    pdp = new PowerDistribution(0, ModuleType.kCTRE);

    Shuffleboard.getTab("pdp").add("PDP", pdp).withWidget(BuiltInWidgets.kPowerDistribution);
    SWERVE.setDefaultCommand(SWERVE.joystickDrive(xBox::getLeftX, xBox::getLeftY, xBox::getRightX, SWERVE));
    configureBindings();
  }

  private void configureBindings() {
    xBox.b().toggleOnTrue(SWERVE.toggleSlowMode());
  }

  public Command getAutonomousCommand() {
    return new Command() {
      
    };
  }
}