// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot implements Loggable {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  SwerveDrive SWERVE;
  Shooter SHOOTER;
  DriveTrain DRIVETRAIN;
  static XboxController controller = new XboxController(0);
  static GenericHID sController = new GenericHID(1);
  static GenericHID tController = new GenericHID(2);

  @Override
  public void robotInit() {

    SWERVE = SwerveDrive.getInstance();
    SHOOTER = Shooter.getInstance();
    DRIVETRAIN = DriveTrain.getInstance();
    
    Logger.configureLoggingAndConfig(this, false);
  }

  @Override
  public void robotPeriodic() {
    Logger.updateEntries();
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    SWERVE.periodic();
    SHOOTER.periodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
