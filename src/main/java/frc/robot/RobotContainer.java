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

/**
 * Central wiring class for the robot. It creates subsystems, controllers, and
 * button bindings so {@link Robot} can stay focused on lifecycle methods.
 */
public class RobotContainer {
  /** Driver controller used for direct robot controls. */
  private final CommandXboxController m_driverController = new CommandXboxController(Constants.kDriverID);
  /** Operator controller used for secondary actions and conditional choices. */
  private final CommandXboxController m_operatorController = new CommandXboxController(Constants.kOperatorID);

  /** Subsystem that controls the Kraken/TalonFX motor. */
  private final KrakenSubsystem m_kraken = new KrakenSubsystem(Constants.Kraken.kMotorID);
  /** Subsystem that controls the Spark MAX motor. */
  private final SparkMaxSubsystem m_sparkMax = new SparkMaxSubsystem(Constants.SparkMax.kMotorID);

  /** Creates the robot container and installs all controller bindings. */
  public RobotContainer() {
    configureBindings();
  }

  /** Maps controller buttons and triggers to commands. */
  private void configureBindings() {
    // Holding the driver's left trigger sends the Kraken to 85 rotations/units;
    // releasing it sends the motor back to the zero position.
    m_driverController.leftTrigger()
        .onTrue(m_kraken.setPositionCmd(85.0))
        .onFalse(m_kraken.setPositionCmd(0.0));

    // Holding the driver's right trigger sends the Spark MAX to 85 rotations/units;
    // releasing it sends the motor back to the zero position.
    m_driverController.rightTrigger()
        .onTrue(m_sparkMax.setPositionCmd(85.0))
        .onFalse(m_sparkMax.setPositionCmd(0.0));

    // The left bumper chooses which subsystem moves based on the operator A button.
    // If A is pressed, the Kraken command runs; otherwise, the Spark MAX command runs.
    m_driverController.leftBumper()
        .onTrue(new ConditionalCommand(m_kraken.setPositionCmd(50.0), 
                                        m_sparkMax.setPositionCmd(50.0), 
                                          m_operatorController.a()::getAsBoolean));

    // Same functions, different method:
    // the triggers below show how to store button conditions in variables before
    // assigning commands. This is useful when a condition is reused in many places.
    Trigger Kraken = new Trigger(m_driverController.leftTrigger()::getAsBoolean);
    Trigger SparkMax = new Trigger(m_driverController.rightTrigger()::getAsBoolean);
    Trigger conditional = new Trigger(m_driverController.leftBumper()::getAsBoolean);
    BooleanSupplier a = m_operatorController.a()::getAsBoolean;

    // Stored trigger version of the left-trigger Kraken binding above.
    Kraken.onTrue(m_kraken.setPositionCmd(85.0))
          .onFalse(m_kraken.setPositionCmd(0.0));

    // Stored trigger version of the right-trigger Spark MAX binding above.
    SparkMax.onTrue(m_sparkMax.setPositionCmd(85.0))
            .onFalse(m_sparkMax.setPositionCmd(0.0));

    // Stored trigger version of the conditional left-bumper binding above.
    conditional
        .onTrue(new ConditionalCommand(m_kraken.setPositionCmd(50.0), m_sparkMax.setPositionCmd(50.0), a));

    /*
    conditional
        .onTrue(m_operatorController.a().getAsBoolean() ? m_kraken.setPositionCmd(50.0) : m_sparkMax.setPositionCmd(50.0));
    */
  }

  /**
   * Returns the command to run during autonomous mode. A zero-second wait is a
   * placeholder that ends immediately.
   */
  public Command getAutonomousCommand() {
    return new WaitCommand(0.0);
  }
}
