// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;

public class Robot extends TimedRobot implements Loggable{
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private GenericEntry matchTimeEntry;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    Logger.configureLoggingAndConfig(this, false);
    matchTimeEntry = Shuffleboard.getTab("selector")
      .add("Match time", "")
      .withWidget(BuiltInWidgets.kTextView)
      .withProperties(Map.of("Width", 50, "Height", 12))
      .withPosition(1, 1)
      .withSize(7,3)
      .getEntry();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Logger.updateEntries();
    matchTimeEntry.setString(String.format("%.2f", DriverStation.getMatchTime()));
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    m_autonomousCommand.schedule();
    
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
