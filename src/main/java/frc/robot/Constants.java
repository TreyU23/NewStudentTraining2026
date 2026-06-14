package frc.robot;

/**
 * Stores robot-wide values that should be reused instead of typed directly into
 * subsystem or command code.
 */
public class Constants {
    /** USB port for the driver Xbox controller. */
    public static final int kDriverID = 0;
    /** USB port for the operator Xbox controller. */
    public static final int kOperatorID = 1;

    /** Constants used by the TalonFX/Kraken motor subsystem. */
    public class Kraken {
        /** CAN ID assigned to the Kraken/TalonFX motor controller. */
        public static final int kMotorID = 1;
        /** Maximum stator current the motor controller should allow. */
        public static final double kStatorCurrent = 50.0;
        /** Maximum supply current pulled from the robot battery. */
        public static final double kSupplyCurrent = 20.0;

        /** Proportional gain for the TalonFX position controller. */
        public static final double kP = 1.0;
        /** Integral gain for the TalonFX position controller. */
        public static final double kI = 0.0;
        /** Derivative gain for the TalonFX position controller. */
        public static final double kD = 0.0;

        /** Highest allowed sensor position before the forward soft limit stops motion. */
        public static final double kForwardLimit = 100.0;
        /** Lowest allowed sensor position before the reverse soft limit stops motion. */
        public static final double kReverseLimit = 0.0;

        /** Acceptable position error used when checking if the motor reached its goal. */
        public static final double kTolerance = 0.15;
    }

    /** Constants used by the REV Spark MAX motor subsystem. */
    public class SparkMax {
        /** CAN ID assigned to the Spark MAX motor controller. */
        public static final int kMotorID = 2;
        /** Maximum current the Spark MAX should allow. */
        public static final double kStatorCurrent = 50.0;

        /** Proportional gain for the Spark MAX position controller. */
        public static final double kP = 1.0;
        /** Integral gain for the Spark MAX position controller. */
        public static final double kI = 0.0;
        /** Derivative gain for the Spark MAX position controller. */
        public static final double kD = 0.0;

        /** Highest allowed encoder position before the forward soft limit stops motion. */
        public static final double kForwardLimit = 100.0;
        /** Lowest allowed encoder position before the reverse soft limit stops motion. */
        public static final double kReverseLimit = 0.0;

        /** Acceptable position error used when checking if the motor reached its goal. */
        public static final double kTolerance = 0.15;
    }
}
