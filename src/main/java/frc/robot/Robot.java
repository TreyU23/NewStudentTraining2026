// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Main robot lifecycle class. WPILib calls these methods as the robot changes
 * between disabled, autonomous, teleop, and test modes.
 */
public class Robot extends TimedRobot {
  /** Command currently running during autonomous mode, if one was supplied. */
  private Command m_autonomousCommand;

  /** Creates and owns subsystems, commands, and controller bindings. */
  private final RobotContainer m_robotContainer;

  /** Builds the robot container once when the robot program starts. */
  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  /** Runs once every robot loop, no matter which mode the robot is in. */
  @Override
  public void robotPeriodic() {
    // The command scheduler polls buttons, starts commands, runs active commands,
    // and calls subsystem periodic methods.
    CommandScheduler.getInstance().run();
  }

  /** Called once when the robot enters disabled mode. */
  @Override
  public void disabledInit() {}

  /** Called repeatedly while the robot is disabled. */
  @Override
  public void disabledPeriodic() {}

  /** Called once when the robot leaves disabled mode. */
  @Override
  public void disabledExit() {}

  /** Called once when autonomous mode starts. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Schedule the selected autonomous command only if one exists.
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** Called repeatedly while autonomous mode is active. */
  @Override
  public void autonomousPeriodic() {}

  /** Called once when autonomous mode ends. */
  @Override
  public void autonomousExit() {}

  /** Called once when teleoperated mode starts. */
  @Override
  public void teleopInit() {
    // Make sure autonomous code does not keep running after drivers take control.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** Called repeatedly while teleoperated mode is active. */
  @Override
  public void teleopPeriodic() {}

  /** Called once when teleoperated mode ends. */
  @Override
  public void teleopExit() {}

  /** Called once when test mode starts. */
  @Override
  public void testInit() {
    // Clear running commands so tests start from a known command state.
    CommandScheduler.getInstance().cancelAll();
  }

  /** Called repeatedly while test mode is active. */
  @Override
  public void testPeriodic() {}

  /** Called once when test mode ends. */
  @Override
  public void testExit() {}
}
