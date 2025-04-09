package org.steelhawks.subsystems.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.steelhawks.Constants;

public class ClimbConstants {

    public enum DeepClimbState {
        HOME(0, Math.PI / 2.0, 0),
        PREPARE(0, 0, 0),
        CLIMB(0, 0, 0);

        private final double alphaRadians;
        private final double omegaRadians;
        private final double hawkriderRadians;

        DeepClimbState(double alpha, double omega, double hawkrider) {
            this.alphaRadians = alpha;
            this.omegaRadians = omega;
            this.hawkriderRadians = hawkrider;
        }

        public Rotation2d getAngle() {
            double angleRads = switch (Constants.getRobot()) {
                case ALPHABOT -> alphaRadians;
                case OMEGABOT -> omegaRadians;
                case HAWKRIDER -> hawkriderRadians;
                default -> 0;
            };
            return new Rotation2d(angleRads);
        }
    }

    // -------------------- SHALLOW --------------------
    public static final int SHALLOW_MOTOR_ID;

    public static final double SHALLOW_GEAR_RATIO;

    public static final double SHALLOW_MAX_VELOCITY_PER_SEC;
    public static final double SHALLOW_MAX_ACCELERATION_PER_SEC_SQUARED;

    public static final double SHALLOW_TOLERANCE;

    public static final double SHALLOW_MAX_RADIANS;

    // -------------------- DEEP --------------------
    public static final int DEEP_TOP_MOTOR_ID;
    public static final int DEEP_BOTTOM_MOTOR_ID;
    public static final int DEEP_CANCODER_ID;

    public static final double DEEP_KP;
    public static final double DEEP_KI;
    public static final double DEEP_KD;

    public static final double DEEP_KS;
    public static final double DEEP_KG;
    public static final double DEEP_KV;

    public static final double CLIMBING_DEEP_KP;
    public static final double CLIMBING_DEEP_KI;
    public static final double CLIMBING_DEEP_KD;

    public static final double CLIMBING_DEEP_KS;
    public static final double CLIMBING_DEEP_KG;
    public static final double CLIMBING_DEEP_KV;

    public static final double DEEP_MAX_VELO_PER_SECOND;
    public static final double DEEP_MAX_ACCEL_PER_SECOND;

    public static final double ANGLE_OFFSET_TO_HORIZONTAL = -0.305908203125;
    public static final double TOLERANCE = Units.degreesToRadians(10);

    static {
        switch (Constants.getRobot()) {
            case ALPHABOT -> {
                SHALLOW_MOTOR_ID = 21;
                SHALLOW_GEAR_RATIO = 1;
                SHALLOW_MAX_VELOCITY_PER_SEC = 8;
                SHALLOW_MAX_ACCELERATION_PER_SEC_SQUARED = 0.005;
                SHALLOW_TOLERANCE = 0.5;
                SHALLOW_MAX_RADIANS = Units.rotationsToRadians(3);
                DEEP_TOP_MOTOR_ID = 30;
                DEEP_BOTTOM_MOTOR_ID = 36;
                DEEP_CANCODER_ID = 22;
                DEEP_KP = 0.0;
                DEEP_KI = 0.0;
                DEEP_KD = 0.0;
                DEEP_KS = 0.0;
                DEEP_KG = 0.0;
                DEEP_KV = 0.0;
                DEEP_MAX_VELO_PER_SECOND = 0.0;
                DEEP_MAX_ACCEL_PER_SECOND = 0.0;
            }
            case HAWKRIDER -> {
                SHALLOW_MOTOR_ID = 21;
                SHALLOW_GEAR_RATIO = 0.0;
                SHALLOW_MAX_VELOCITY_PER_SEC = 0.0;
                SHALLOW_MAX_ACCELERATION_PER_SEC_SQUARED = 0.0;
                SHALLOW_TOLERANCE = 0.0;
                SHALLOW_MAX_RADIANS = 0.0;
                DEEP_TOP_MOTOR_ID = 0;
                DEEP_BOTTOM_MOTOR_ID = 0;
                DEEP_CANCODER_ID = 0;
                DEEP_KP = 0.0;
                DEEP_KI = 0.0;
                DEEP_KD = 0.0;
                DEEP_KS = 0.0;
                DEEP_KG = 0.0;
                DEEP_KV = 0.0;
                DEEP_MAX_VELO_PER_SECOND = 0.0;
                DEEP_MAX_ACCEL_PER_SECOND = 0.0;
            }
            default -> {
                SHALLOW_MOTOR_ID = 21;
                SHALLOW_GEAR_RATIO = 1.0;
                SHALLOW_MAX_VELOCITY_PER_SEC = 8.0;
                SHALLOW_MAX_ACCELERATION_PER_SEC_SQUARED = 12.0;
                SHALLOW_TOLERANCE = 0.05;
                SHALLOW_MAX_RADIANS = Units.rotationsToRadians(3.0);
                DEEP_TOP_MOTOR_ID = 30;
                DEEP_BOTTOM_MOTOR_ID = 36;
                DEEP_CANCODER_ID = 18; // 18
                DEEP_KP = 20.0;
                DEEP_KI = 0.0;
                DEEP_KD = 0.0;
                DEEP_KS = 0.23;
                DEEP_KG = 0.0;
                DEEP_KV = 12.0;
                DEEP_MAX_VELO_PER_SECOND = 20.0;
                DEEP_MAX_ACCEL_PER_SECOND = 25.0;
            }
        }

        // deep is only on OMEGABOT
        CLIMBING_DEEP_KP = 0.0;
        CLIMBING_DEEP_KI = 0.0;
        CLIMBING_DEEP_KD = 0.0;
        CLIMBING_DEEP_KS = 0.0;
        CLIMBING_DEEP_KG = 0.0;
        CLIMBING_DEEP_KV = 0.0;
    }
}
