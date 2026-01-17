package org.steelhawks.subsystems.arm;

import org.steelhawks.Constants;

public class ArmConstants {

    public enum ArmState {
        HOME(-Math.PI / 2, -Math.PI / 2, 0.0),
        ERECT(0, 0, 0),
        AVOID_ELEVATOR(0, -Math.PI / 4,0);

        private final double alphaRadians;
        private final double omegaRadians;
        private final double hawkriderRadians;

        ArmState(double alpha, double omega, double hawkrider) {
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

    public static final int ARM_SPIN_MOTOR_ID;
    public static final int ARM_PIVOT_MOTOR_ID;
    public static final int ARM_LIMIT_SWITCH_ID;
    public static final int ARM_CANCODER_ID;

    public static final double ARM_SPIN_GEAR_RATIO;
    public static final double ARM_PIVOT_GEAR_RATIO;

    public static final double ARM_KS;
    public static final double ARM_KG;
    public static final double ARM_KV;

    public static final double ARM_KP;
    public static final double ARM_KI;
    public static final double ARM_KD;

    public static final double ARM_SPEED_MULTIPLIER;
    public static final double ARM_MAX_VELOCITY_PER_SEC;
    public static final double ARM_MAX_ACCELERATION_PER_SEC_SQUARED;
    public static final double ARM_TOLERANCE;
    public static final double ARM_MANUAL_PIVOT_INCREMENT;
    public static final double ARM_MAX_RADIANS;
    public static final double ARM_MIN_RADIANS;

    static {
        switch (Constants.getRobot()) {
            case ALPHABOT, HAWKRIDER -> {
                ARM_SPIN_MOTOR_ID = 0;
                ARM_PIVOT_MOTOR_ID = 0;
                ARM_LIMIT_SWITCH_ID = -1;
                ARM_CANCODER_ID = -1;
                ARM_SPIN_GEAR_RATIO = 0.0;
                ARM_PIVOT_GEAR_RATIO = 0.0;
                ARM_KS = 0.0;
                ARM_KG = 0.0;
                ARM_KV = 0.0;
                ARM_KP = 0.0;
                ARM_KI = 0.0;
                ARM_KD = 0.0;
                ARM_SPEED_MULTIPLIER = 0.0;
                ARM_MAX_VELOCITY_PER_SEC = 0.0;
                ARM_MAX_ACCELERATION_PER_SEC_SQUARED = 0.0;
                ARM_TOLERANCE = 0.0;
                ARM_MANUAL_PIVOT_INCREMENT = 0.0;
                ARM_MAX_RADIANS = 0.0;
                ARM_MIN_RADIANS = 0.0;
            }
            default -> {
                ARM_SPIN_MOTOR_ID = 33;
                ARM_PIVOT_MOTOR_ID = 23;
                ARM_LIMIT_SWITCH_ID = -1;
                ARM_CANCODER_ID = 25;
                ARM_SPIN_GEAR_RATIO = 1.0;
                ARM_PIVOT_GEAR_RATIO = 1.0;
                ARM_KS = 0.0;
                ARM_KG = 0.7;
                ARM_KV = 0.0;
                ARM_KP = 0.0;
                ARM_KI = 0.0;
                ARM_KD = 0.0;
                ARM_SPEED_MULTIPLIER = 1.0;
                ARM_MAX_VELOCITY_PER_SEC = 5.2;
                ARM_MAX_ACCELERATION_PER_SEC_SQUARED = 8;
                ARM_TOLERANCE = 0.002;
                ARM_MANUAL_PIVOT_INCREMENT = 0.1;
                ARM_MAX_RADIANS = Math.PI;
                ARM_MIN_RADIANS = -Math.PI / 2;
            }
        }
    }
}
