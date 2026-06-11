package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
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

public class KrakenSubsystem extends SubsystemBase { 
    private final TalonFX m_motor;

    private final PositionVoltage m_request 
            = new PositionVoltage(0.0).withEnableFOC(true);

    private static double m_setpoint = 0.0;
    private static boolean m_allowedMovement = false;
    
    public KrakenSubsystem(int CanID) {
        m_motor = new TalonFX(CanID);

        motorConfigs(m_motor.getConfigurator());
    }

    public double getStatorCurrent() {
        return m_motor.getStatorCurrent().getValueAsDouble();
    }

    public double getSupplyCurrent() {
        return m_motor.getSupplyCurrent().getValueAsDouble();
    }

    public double getSetpoint() {
        return m_setpoint;
    }

    public double getPosition() {
        return m_motor.getPosition().getValueAsDouble();
    }

    public boolean atSetpoint() {
        if (Math.abs(getPosition()-m_setpoint) < Kraken.kTolerance) return true;
        else return false;
    }

    public void setPosition(double setpoint) {
        m_setpoint = setpoint;
        if (m_allowedMovement)
            m_motor.setControl(m_request.withPosition(m_setpoint));
    }

    public Command setPositionCmd(double setpoint) {
        return run(()-> setPosition(setpoint));
    }

    public void stop() {
        m_motor.stopMotor();
    }

    public Command stopCmd() {
        return runOnce(()-> stop());
    }

    public void allowedToMove(boolean allowed) {
        m_allowedMovement = allowed;
    }

    @Override
    public void periodic() {
        String subsystem = getName();

        SmartDashboard.putNumber(subsystem + " setpoint", m_setpoint);
        SmartDashboard.putNumber(subsystem + " position", getPosition());
        SmartDashboard.putNumber(subsystem + " stator current", getStatorCurrent());
        SmartDashboard.putNumber(subsystem + " supply current", getSupplyCurrent());
        SmartDashboard.putBoolean(subsystem + " allowed to move", m_allowedMovement);
    }

    private void motorConfigs(TalonFXConfigurator config) {
        config.apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Kraken.kStatorCurrent)
            .withSupplyCurrentLimit(Kraken.kSupplyCurrent)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true));

        config.apply(new Slot0Configs()
            .withKP(Kraken.kP)
            .withKI(Kraken.kI)
            .withKD(Kraken.kD)
            .withGravityType(GravityTypeValue.Arm_Cosine));

        config.apply(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(Kraken.kForwardLimit)
            .withReverseSoftLimitThreshold(Kraken.kReverseLimit)
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true));

        config.apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));
    }
}