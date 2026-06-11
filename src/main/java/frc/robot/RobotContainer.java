// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.KrakenSubsystem;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(Constants.kDriverID);
  private final CommandXboxController m_operatorController = new CommandXboxController(Constants.kOperatorID);

  private final KrakenSubsystem m_kraken = new KrakenSubsystem(Constants.Kraken.kMotorID);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return new WaitCommand(0.0);
  }
}