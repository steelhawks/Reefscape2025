package org.steelhawks.subsystems.schlong;

import org.steelhawks.Constants;

public class SchlongConstants {

    public enum SchlongState {
        HOME(-Math.PI / 2, -Math.PI / 2, 0.0),
        ERECT(0, 0, 0),
        AVOID_ELEVATOR(0, -Math.PI / 4,0);

        private final double alphaRadians;
        private final double omegaRadians;
        private final double hawkriderRadians;

        SchlongState(double alpha, double omega, double hawkrider) {
            this.alphaRadians = alpha;
            this.omegaRadians = omega;
            this.hawkriderRadians = hawkrider;
        }

        public double getRadians() {
            switch (Constants.getRobot()) {
                case ALPHABOT:
                    return alphaRadians;
                case OMEGABOT:
                    return omegaRadians;
                case HAWKRIDER:
                    return hawkriderRadians;
                default:
                    return 0;
            }
        }
    }

    public static final int SCHLONG_SPIN_MOTOR_ID;
    public static final int SCHLONG_PIVOT_MOTOR_ID;
    public static final int SCHLONG_LIMIT_SWITCH_ID;
    public static final int SCHLONG_CANCODER_ID;

    public static final double SCHLONG_SPIN_GEAR_RATIO;
    public static final double SCHLONG_PIVOT_GEAR_RATIO;

    public static final double SCHLONG_KS;
    public static final double SCHLONG_KG;
    public static final double SCHLONG_KV;

    public static final double SCHLONG_KP;
    public static final double SCHLONG_KI;
    public static final double SCHLONG_KD;

    public static final double SCHLONG_SPEED_MULTIPLIER;
    public static final double SCHLONG_MAX_VELOCITY_PER_SEC;
    public static final double SCHLONG_MAX_ACCELERATION_PER_SEC_SQUARED;
    public static final double SCHLONG_TOLERANCE;
    public static final double SCHLONG_MANUAL_PIVOT_INCREMENT;
    public static final double SCHLONG_MAX_RADIANS;
    public static final double SCHLONG_MIN_RADIANS;

    static {
        switch (Constants.getRobot()) {
            case ALPHABOT, HAWKRIDER -> {
                SCHLONG_SPIN_MOTOR_ID = 0;
                SCHLONG_PIVOT_MOTOR_ID = 0;
                SCHLONG_LIMIT_SWITCH_ID = -1;
                SCHLONG_CANCODER_ID = -1;
                SCHLONG_SPIN_GEAR_RATIO = 0.0;
                SCHLONG_PIVOT_GEAR_RATIO = 0.0;
                SCHLONG_KS = 0.0;
                SCHLONG_KG = 0.0;
                SCHLONG_KV = 0.0;
                SCHLONG_KP = 0.0;
                SCHLONG_KI = 0.0;
                SCHLONG_KD = 0.0;
                SCHLONG_SPEED_MULTIPLIER = 0.0;
                SCHLONG_MAX_VELOCITY_PER_SEC = 0.0;
                SCHLONG_MAX_ACCELERATION_PER_SEC_SQUARED = 0.0;
                SCHLONG_TOLERANCE = 0.0;
                SCHLONG_MANUAL_PIVOT_INCREMENT = 0.0;
                SCHLONG_MAX_RADIANS = 0.0;
                SCHLONG_MIN_RADIANS = 0.0;
            }
            default -> {
                SCHLONG_SPIN_MOTOR_ID = 33;
                SCHLONG_PIVOT_MOTOR_ID = 23;
                SCHLONG_LIMIT_SWITCH_ID = -1;
                SCHLONG_CANCODER_ID = 25;
                SCHLONG_SPIN_GEAR_RATIO = 1.0;
                SCHLONG_PIVOT_GEAR_RATIO = 1.0;
                SCHLONG_KS = 0.0;
                SCHLONG_KG = 0.7;
                SCHLONG_KV = 0.0;
                SCHLONG_KP = 0.0;
                SCHLONG_KI = 0.0;
                SCHLONG_KD = 0.0;
                SCHLONG_SPEED_MULTIPLIER = 1.0;
                SCHLONG_MAX_VELOCITY_PER_SEC = 5.2;
                SCHLONG_MAX_ACCELERATION_PER_SEC_SQUARED = 8;
                SCHLONG_TOLERANCE = 0.002;
                SCHLONG_MANUAL_PIVOT_INCREMENT = 0.1;
                SCHLONG_MAX_RADIANS = Math.PI;
                SCHLONG_MIN_RADIANS = -Math.PI / 2;
            }
        }
    }
}
