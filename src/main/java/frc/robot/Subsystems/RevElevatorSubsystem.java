package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RevElevator;

/** Basic elevator using a REV Spark MAX and onboard MAXMotion position control. */
public class RevElevatorSubsystem extends SubsystemBase {
    private final SparkMax m_motor;
    private final RelativeEncoder m_encoder;
    private final SparkClosedLoopController m_controller;

    private double m_setpoint = RevElevator.kHomeHeight;
    private boolean m_allowedMovement = false;

    public RevElevatorSubsystem(int canID) {
        m_motor = new SparkMax(canID, MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();
        m_controller = m_motor.getClosedLoopController();
        motorConfigs(m_motor);
        zeroPosition();
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public double getSetpoint() {
        return m_setpoint;
    }

    public boolean isAllowedToMove() {
        return m_allowedMovement;
    }

    public boolean atSetpoint() {
        return Math.abs(getPosition() - m_setpoint) <= RevElevator.kTolerance;
    }

    public void zeroPosition() {
        m_encoder.setPosition(RevElevator.kHomeHeight);
        m_setpoint = RevElevator.kHomeHeight;
    }

    public void allowMovement(boolean allowed) {
        m_allowedMovement = allowed;
        if (!allowed) {
            stop();
        }
    }

    public void setHeight(double height) {
        m_setpoint = MathUtil.clamp(height, RevElevator.kReverseLimit, RevElevator.kForwardLimit);
        if (m_allowedMovement) {
            m_controller.setSetpoint(m_setpoint, ControlType.kMAXMotionPositionControl);
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
        SmartDashboard.putNumber(subsystem + " current", m_motor.getOutputCurrent());
        SmartDashboard.putBoolean(subsystem + " allowed to move", m_allowedMovement);
        SmartDashboard.putBoolean(subsystem + " at setpoint", atSetpoint());
    }

    private void motorConfigs(SparkMax motor) {
        SparkMaxConfig config = new SparkMaxConfig();

        config.smartCurrentLimit(RevElevator.kCurrentLimit);

        config.encoder.positionConversionFactor(RevElevator.kPositionConversionFactor)
            .velocityConversionFactor(RevElevator.kVelocityConversionFactor);

        config.closedLoop
            .pid(RevElevator.kP, RevElevator.kI, RevElevator.kD)
            .outputRange(RevElevator.kMinOutput, RevElevator.kMaxOutput);

        config.closedLoop.maxMotion
            .cruiseVelocity(RevElevator.kCruiseVelocity)
            .maxAcceleration(RevElevator.kAcceleration)
            .allowedProfileError(RevElevator.kAllowedProfileError);

        config.softLimit
            .forwardSoftLimit(RevElevator.kForwardLimit)
            .reverseSoftLimit(RevElevator.kReverseLimit)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);

        config.inverted(false).idleMode(IdleMode.kBrake);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
