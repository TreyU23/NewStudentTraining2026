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

    /** Constants used by the example CTRE Motion Magic elevator subsystem. */
    public class CtreElevator {
        /** CAN ID assigned to the elevator TalonFX/Kraken motor controller. */
        public static final int kMotorID = 3;
        /** Lowest allowed elevator height in motor rotations/mechanism units. */
        public static final double kHomeHeight = 0.0;
        /** Safe low scoring height. */
        public static final double kLowHeight = 10.0;
        /** Safe mid scoring height. */
        public static final double kMidHeight = 35.0;
        /** Safe high scoring height. */
        public static final double kHighHeight = 70.0;

        /** Maximum stator current the motor controller should allow. */
        public static final double kStatorCurrent = 40.0;
        /** Maximum supply current pulled from the robot battery. */
        public static final double kSupplyCurrent = 20.0;

        /** Position gain for Motion Magic. */
        public static final double kP = 1.0;
        /** Integral gain for Motion Magic. */
        public static final double kI = 0.0;
        /** Derivative gain for Motion Magic. */
        public static final double kD = 0.0;
        /** Static elevator gravity feedforward in volts. Tune on the real robot. */
        public static final double kG = 0.0;

        /** Motion Magic cruise velocity in rotations per second/mechanism units per second. */
        public static final double kCruiseVelocity = 40.0;
        /** Motion Magic acceleration in rotations per second squared/mechanism units per second squared. */
        public static final double kAcceleration = 80.0;
        /** Motion Magic jerk in rotations per second cubed/mechanism units per second cubed. */
        public static final double kJerk = 800.0;

        /** Highest allowed elevator height before the forward soft limit stops motion. */
        public static final double kForwardLimit = 75.0;
        /** Lowest allowed elevator height before the reverse soft limit stops motion. */
        public static final double kReverseLimit = 0.0;
        /** Acceptable position error used when checking if the elevator reached its goal. */
        public static final double kTolerance = 0.25;
    }

    /** Constants used by the example REV MAXMotion elevator subsystem. */
    public class RevElevator {
        /** CAN ID assigned to the elevator Spark MAX motor controller. */
        public static final int kMotorID = 4;
        /** Lowest allowed elevator height in motor rotations/mechanism units. */
        public static final double kHomeHeight = 0.0;
        /** Safe low scoring height. */
        public static final double kLowHeight = 10.0;
        /** Safe mid scoring height. */
        public static final double kMidHeight = 35.0;
        /** Safe high scoring height. */
        public static final double kHighHeight = 70.0;

        /** Maximum current the Spark MAX should allow. */
        public static final int kCurrentLimit = 40;
        /** Position conversion factor from motor rotations to elevator units. */
        public static final double kPositionConversionFactor = 1.0;
        /** Velocity conversion factor from motor RPM to elevator units per minute. */
        public static final double kVelocityConversionFactor = 1.0;

        /** Position gain for MAXMotion position control. */
        public static final double kP = 0.05;
        /** Integral gain for MAXMotion position control. */
        public static final double kI = 0.0;
        /** Derivative gain for MAXMotion position control. */
        public static final double kD = 0.0;

        /** Smallest output allowed while the Spark MAX follows a MAXMotion profile. */
        public static final double kMinOutput = -1.0;
        /** Largest output allowed while the Spark MAX follows a MAXMotion profile. */
        public static final double kMaxOutput = 1.0;
        /** MAXMotion cruise velocity in elevator units per minute. */
        public static final double kCruiseVelocity = 2400.0;
        /** MAXMotion acceleration in elevator units per minute per second. */
        public static final double kAcceleration = 4800.0;
        /** Allowed profile error before MAXMotion regenerates its profile. */
        public static final double kAllowedProfileError = 1.0;

        /** Highest allowed elevator height before the forward soft limit stops motion. */
        public static final double kForwardLimit = 75.0;
        /** Lowest allowed elevator height before the reverse soft limit stops motion. */
        public static final double kReverseLimit = 0.0;
        /** Acceptable position error used when checking if the elevator reached its goal. */
        public static final double kTolerance = 0.25;
    }

}
