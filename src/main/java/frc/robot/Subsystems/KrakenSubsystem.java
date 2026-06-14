package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Kraken;

/** Controls a TalonFX/Kraken motor using closed-loop position control. */
public class KrakenSubsystem extends SubsystemBase { 
    /** TalonFX motor controller connected over CAN. */
    private final TalonFX m_motor;

    /** Reused position-control request sent to the TalonFX. */
    private final PositionVoltage m_request 
            = new PositionVoltage(0.0).withEnableFOC(true);

    /** Last position goal requested for the subsystem. */
    private static double m_setpoint = 0.0;
    /** Safety flag that must be true before position requests are sent to the motor. */
    private static boolean m_allowedMovement = false;
    
    /** Creates the subsystem and configures the motor controller. */
    public KrakenSubsystem(int CanID) {
        m_motor = new TalonFX(CanID);

        motorConfigs(m_motor.getConfigurator());
    }

    /** Returns the current flowing through the motor windings. */
    public double getStatorCurrent() {
        return m_motor.getStatorCurrent().getValueAsDouble();
    }

    /** Returns the current being drawn from the robot power supply. */
    public double getSupplyCurrent() {
        return m_motor.getSupplyCurrent().getValueAsDouble();
    }

    /** Returns the current target position for the subsystem. */
    public double getSetpoint() {
        return m_setpoint;
    }

    /** Returns the motor controller's measured position. */
    public double getPosition() {
        return m_motor.getPosition().getValueAsDouble();
    }

    /** Returns true when the measured position is close enough to the setpoint. */
    public boolean atSetpoint() {
        if (Math.abs(getPosition()-m_setpoint) < Kraken.kTolerance) return true;
        else return false;
    }

    /**
     * Updates the target position and sends it to the motor only when movement
     * has been enabled.
     */
    public void setPosition(double setpoint) {
        m_setpoint = setpoint;
        if (m_allowedMovement)
            m_motor.setControl(m_request.withPosition(m_setpoint));
    }

    /** Creates a command that continuously requests the given position while scheduled. */
    public Command setPositionCmd(double setpoint) {
        return run(()-> setPosition(setpoint));
    }

    /** Immediately stops motor output. */
    public void stop() {
        m_motor.stopMotor();
    }

    /** Creates a one-shot command that stops the motor. */
    public Command stopCmd() {
        return runOnce(()-> stop());
    }

    /** Enables or disables whether setPosition is allowed to command the motor. */
    public void allowedToMove(boolean allowed) {
        m_allowedMovement = allowed;
    }

    /** Publishes live subsystem values to SmartDashboard each scheduler loop. */
    @Override
    public void periodic() {
        String subsystem = getName();

        SmartDashboard.putNumber(subsystem + " setpoint", m_setpoint);
        SmartDashboard.putNumber(subsystem + " position", getPosition());
        SmartDashboard.putNumber(subsystem + " stator current", getStatorCurrent());
        SmartDashboard.putNumber(subsystem + " supply current", getSupplyCurrent());
        SmartDashboard.putBoolean(subsystem + " allowed to move", m_allowedMovement);
    }

    /** Applies all TalonFX configuration used by this subsystem. */
    private void motorConfigs(TalonFXConfigurator config) {
        config.apply(new TalonFXConfiguration());
            // Reset the controller to safe default parameters before applying custom settings.

        // Limit motor and battery current to protect the mechanism and electrical system.
        config.apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Kraken.kStatorCurrent)
            .withSupplyCurrentLimit(Kraken.kSupplyCurrent)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true));

        // Tune slot 0 for position control and apply arm gravity compensation.
        config.apply(new Slot0Configs()
            .withKP(Kraken.kP)
            .withKI(Kraken.kI)
            .withKD(Kraken.kD)
            .withGravityType(GravityTypeValue.Arm_Cosine));

        // Prevent closed-loop commands from driving past the configured range.
        config.apply(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(Kraken.kForwardLimit)
            .withReverseSoftLimitThreshold(Kraken.kReverseLimit)
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true));

        // Set motor direction and make the motor brake instead of coast when idle.
        config.apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));
    }
}
