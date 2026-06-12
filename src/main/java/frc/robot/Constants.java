package frc.robot;

public class Constants {
    public static final int kDriverID = 0;
    public static final int kOperatorID = 1;

    public class Kraken {
        public static final int kMotorID = 1;
        public static final double kStatorCurrent = 50.0;
        public static final double kSupplyCurrent = 20.0;

        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kForwardLimit = 100.0;
        public static final double kReverseLimit = 0.0;

        public static final double kTolerance = 0.15;
    }

    public class SparkMax {
        public static final int kMotorID = 2;
        public static final double kStatorCurrent = 50.0;

        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kForwardLimit = 100.0;
        public static final double kReverseLimit = 0.0;

        public static final double kTolerance = 0.15;
    }
}
