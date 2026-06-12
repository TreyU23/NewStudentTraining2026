// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.KrakenSubsystem;
import frc.robot.Subsystems.SparkMaxSubsystem;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(Constants.kDriverID);
  private final CommandXboxController m_operatorController = new CommandXboxController(Constants.kOperatorID);

  private final KrakenSubsystem m_kraken = new KrakenSubsystem(Constants.Kraken.kMotorID);
  private final SparkMaxSubsystem m_sparkMax = new SparkMaxSubsystem(Constants.SparkMax.kMotorID);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.leftTrigger()
        .onTrue(m_kraken.setPositionCmd(85.0))
        .onFalse(m_kraken.setPositionCmd(0.0));

    m_driverController.rightTrigger()
        .onTrue(m_sparkMax.setPositionCmd(85.0))
        .onFalse(m_sparkMax.setPositionCmd(0.0));

    m_driverController.leftBumper()
        .onTrue(new ConditionalCommand(m_kraken.setPositionCmd(50.0), 
                                        m_sparkMax.setPositionCmd(50.0), 
                                          m_operatorController.a()::getAsBoolean));

    //Same Functions Diffrent Methods
    Trigger Kraken = new Trigger(m_driverController.leftTrigger()::getAsBoolean);
    Trigger SparkMax = new Trigger(m_driverController.rightTrigger()::getAsBoolean);
    Trigger conditional = new Trigger(m_driverController.leftBumper()::getAsBoolean);
    BooleanSupplier a = m_operatorController.a()::getAsBoolean;

    Kraken.onTrue(m_kraken.setPositionCmd(85.0))
          .onFalse(m_kraken.setPositionCmd(0.0));

    SparkMax.onTrue(m_sparkMax.setPositionCmd(85.0))
            .onFalse(m_sparkMax.setPositionCmd(0.0));

    conditional
        .onTrue(new ConditionalCommand(m_kraken.setPositionCmd(50.0), m_sparkMax.setPositionCmd(50.0), a));

    /*
    conditional
        .onTrue(m_operatorController.a().getAsBoolean() ? m_kraken.setPositionCmd(50.0) : m_sparkMax.setPositionCmd(50.0));
    */
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(0.0);
  }
}