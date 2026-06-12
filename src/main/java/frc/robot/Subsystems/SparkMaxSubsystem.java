package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SparkMaxSubsystem extends SubsystemBase {
    private final SparkMax m_motor;
    private final RelativeEncoder m_encoder;
    private final SparkClosedLoopController m_pid;
    
    private static double m_setpoint = 0.0;
    private static boolean m_allowedMovement = false; 
    
    public SparkMaxSubsystem(int CanID) {
        m_motor = new SparkMax(CanID, MotorType.kBrushless);

        m_encoder = m_motor.getEncoder();
        m_pid = m_motor.getClosedLoopController();

        motorConfigs(m_motor);
    }

    public double getCurrent() {
        return m_motor.getOutputCurrent();
    }

    public double getSetpoint() {
        return m_setpoint;
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public boolean atSetpoint() {
        if (Math.abs(getPosition()-m_setpoint) < Constants.SparkMax.kTolerance) return true;
        else return false;
    }

    public void setPosition(double setpoint) {
        m_setpoint = setpoint;
        if (m_allowedMovement) 
            m_pid.setSetpoint(m_setpoint, ControlType.kPosition);
    }

    public Command setPositionCmd(double setpoint) {
        return run(()-> setPosition(setpoint));
    }

    public void stop() {
        m_motor.stopMotor();
    }

    public Command stopCmd() {
        return runOnce(()-> stopCmd());
    }

    public void allowedToMove(boolean allowed) {
        m_allowedMovement = allowed;
    }

    @Override
    public void periodic() {
        String subsystem = getName();

        SmartDashboard.putNumber(subsystem + " setpoint", m_setpoint);
        SmartDashboard.putNumber(subsystem + " position", getPosition());
        SmartDashboard.putNumber(subsystem + " stator current", getCurrent());
        SmartDashboard.putBoolean(subsystem + " allowed to move", m_allowedMovement);
    }

    private void motorConfigs(SparkMax motor) {
        SparkMaxConfig config = new SparkMaxConfig();

        config.closedLoop
            .pid(Constants.SparkMax.kP, 
                    Constants.SparkMax.kI, 
                        Constants.SparkMax.kD)
            .outputRange(-100.0, 100.0);

        config.softLimit
            .forwardSoftLimit(Constants.SparkMax.kForwardLimit)
            .reverseSoftLimit(Constants.SparkMax.kReverseLimit)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);

        config.inverted(false).idleMode(IdleMode.kBrake);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}