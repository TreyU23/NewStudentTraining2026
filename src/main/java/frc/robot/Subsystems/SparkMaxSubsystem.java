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

/** Controls a Spark MAX brushless motor using encoder-based position control. */
public class SparkMaxSubsystem extends SubsystemBase {
    /** Spark MAX motor controller connected over CAN. */
    private final SparkMax m_motor;
    /** Relative encoder built into or attached to the Spark MAX motor. */
    private final RelativeEncoder m_encoder;
    /** Closed-loop controller used to send position setpoints to the Spark MAX. */
    private final SparkClosedLoopController m_pid;
    
    /** Last position goal requested for the subsystem. */
    private static double m_setpoint = 0.0;
    /** Safety flag that must be true before position requests are sent to the motor. */
    private static boolean m_allowedMovement = false; 
    
    /** Creates the subsystem, gets motor helper objects, and configures the controller. */
    public SparkMaxSubsystem(int CanID) {
        m_motor = new SparkMax(CanID, MotorType.kBrushless);

        m_encoder = m_motor.getEncoder();
        m_pid = m_motor.getClosedLoopController();

        motorConfigs(m_motor);
    }

    /** Returns the motor output current reported by the Spark MAX. */
    public double getCurrent() {
        return m_motor.getOutputCurrent();
    }

    /** Returns the current target position for the subsystem. */
    public double getSetpoint() {
        return m_setpoint;
    }

    /** Returns the encoder's measured position. */
    public double getPosition() {
        return m_encoder.getPosition();
    }

    /** Returns true when the measured position is close enough to the setpoint. */
    public boolean atSetpoint() {
        if (Math.abs(getPosition()-m_setpoint) < Constants.SparkMax.kTolerance) return true;
        else return false;
    }

    /**
     * Updates the target position and sends it to the motor only when movement
     * has been enabled.
     */
    public void setPosition(double setpoint) {
        m_setpoint = setpoint;
        if (m_allowedMovement) 
            m_pid.setSetpoint(m_setpoint, ControlType.kPosition);
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
        SmartDashboard.putNumber(subsystem + " stator current", getCurrent());
        SmartDashboard.putBoolean(subsystem + " allowed to move", m_allowedMovement);
    }

    /** Applies all Spark MAX configuration used by this subsystem. */
    private void motorConfigs(SparkMax motor) {
        SparkMaxConfig config = new SparkMaxConfig();

        // Configure closed-loop PID gains and the allowed controller output range.
        config.closedLoop
            .pid(Constants.SparkMax.kP, 
                    Constants.SparkMax.kI, 
                        Constants.SparkMax.kD)
            .outputRange(-100.0, 100.0);

        // Prevent closed-loop commands from driving past the configured range.
        config.softLimit
            .forwardSoftLimit(Constants.SparkMax.kForwardLimit)
            .reverseSoftLimit(Constants.SparkMax.kReverseLimit)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);

        // Set motor direction and make the motor brake instead of coast when idle.
        config.inverted(false).idleMode(IdleMode.kBrake);

        // Apply the settings after resetting safe parameters, without saving to flash.
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
