package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CtreElevator;

/** Basic elevator using a CTRE TalonFX/Kraken and onboard Motion Magic. */
public class CtreElevatorSubsystem extends SubsystemBase {
    private final TalonFX m_motor;
    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0.0).withEnableFOC(true);

    private double m_setpoint = CtreElevator.kHomeHeight;
    private boolean m_allowedMovement = false;

    public CtreElevatorSubsystem(int canID) {
        m_motor = new TalonFX(canID);
        motorConfigs(m_motor.getConfigurator());
        zeroPosition();
    }

    public double getPosition() {
        return m_motor.getPosition().getValueAsDouble();
    }

    public double getSetpoint() {
        return m_setpoint;
    }

    public boolean isAllowedToMove() {
        return m_allowedMovement;
    }

    public boolean atSetpoint() {
        return Math.abs(getPosition() - m_setpoint) <= CtreElevator.kTolerance;
    }

    public void zeroPosition() {
        m_motor.setPosition(CtreElevator.kHomeHeight);
        m_setpoint = CtreElevator.kHomeHeight;
    }

    public void allowMovement(boolean allowed) {
        m_allowedMovement = allowed;
        if (!allowed) {
            stop();
        }
    }

    public void setHeight(double height) {
        m_setpoint = MathUtil.clamp(height, CtreElevator.kReverseLimit, CtreElevator.kForwardLimit);
        if (m_allowedMovement) {
            m_motor.setControl(m_request.withPosition(m_setpoint));
        }
    }

    public Command setHeightCmd(double height) {
        return run(() -> setHeight(height));
    }

    public Command allowMovementCmd(boolean allowed) {
        return runOnce(() -> allowMovement(allowed));
    }

    public Command zeroPositionCmd() {
        return runOnce(this::zeroPosition);
    }

    public void stop() {
        m_motor.stopMotor();
    }

    public Command stopCmd() {
        return runOnce(this::stop);
    }

    @Override
    public void periodic() {
        String subsystem = getName();
        SmartDashboard.putNumber(subsystem + " setpoint", m_setpoint);
        SmartDashboard.putNumber(subsystem + " position", getPosition());
        SmartDashboard.putNumber(subsystem + " stator current", m_motor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber(subsystem + " supply current", m_motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putBoolean(subsystem + " allowed to move", m_allowedMovement);
        SmartDashboard.putBoolean(subsystem + " at setpoint", atSetpoint());
    }

    private void motorConfigs(TalonFXConfigurator config) {
        config.apply(new TalonFXConfiguration());

        config.apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(CtreElevator.kStatorCurrent)
            .withSupplyCurrentLimit(CtreElevator.kSupplyCurrent)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true));

        config.apply(new Slot0Configs()
            .withKP(CtreElevator.kP)
            .withKI(CtreElevator.kI)
            .withKD(CtreElevator.kD)
            .withKG(CtreElevator.kG)
            .withGravityType(GravityTypeValue.Elevator_Static));

        config.apply(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(CtreElevator.kCruiseVelocity)
            .withMotionMagicAcceleration(CtreElevator.kAcceleration)
            .withMotionMagicJerk(CtreElevator.kJerk));

        config.apply(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(CtreElevator.kForwardLimit)
            .withReverseSoftLimitThreshold(CtreElevator.kReverseLimit)
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true));

        config.apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));
    }
}
