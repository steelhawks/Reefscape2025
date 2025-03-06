package org.steelhawks.subsystems.climb;

import edu.wpi.first.math.util.Units;
import org.steelhawks.Constants;

public final class ClimbConstants {

    public enum DeepClimbState {
        HOME(0, 0, 0),
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

    public static final ClimbConstants DEFAULT =
        new ClimbConstants(
            21,
            1,
            8,
            0.005,
            0.5,
            Units.rotationsToRadians(3),
            30,
            36,
            -1,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0);

    public static final ClimbConstants OMEGA = DEFAULT;

    public static final ClimbConstants ALPHA =
        new ClimbConstants(
            21,
            1,
            8,
            0.005,
            0.5,
            Units.rotationsToRadians(3),
            30,
            36,
            22,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0);

    public static final ClimbConstants HAWKRIDER = DEFAULT;

    public final int SHALLOW_MOTOR_ID;

    public final double SHALLOW_GEAR_RATIO;

    public final double SHALLOW_MAX_VELOCITY_PER_SEC;
    public final double SHALLOW_MAX_ACCELERATION_PER_SEC_SQUARED;

    public final double SHALLOW_TOLERANCE;

    public final double SHALLOW_MAX_RADIANS;

    public final int DEEP_TOP_MOTOR_ID;
    public final int DEEP_BOTTOM_MOTOR_ID;
    public final int DEEP_CANCODER_ID;

    public final double DEEP_KP;
    public final double DEEP_KI;
    public final double DEEP_KD;

    public final double DEEP_KS;
    public final double DEEP_KG;
    public final double DEEP_KV;

    public final double DEEP_MAX_VELO_PER_SECOND;
    public final double DEEP_MAX_ACCEL_PER_SECOND;

    public ClimbConstants(
        int shallow_motorId,
        double shallow_gearRatio,
        double shallow_maxVelocityPerSec,
        double shallow_maxAccelerationPerSecSquared,
        double shallow_tolerance,
        double shallow_maxRadians,
        int deep_topMotorId,
        int deep_bottomMotorId,
        int deep_cancoderId,
        double deep_kP,
        double deep_kI,
        double deep_kD,
        double deep_kS,
        double deep_kG,
        double deep_kV,
        double deep_maxVelocityPerSecond,
        double deep_maxAccelerationPerSecond
    ) {
        SHALLOW_MOTOR_ID = shallow_motorId;
        SHALLOW_GEAR_RATIO = shallow_gearRatio;
        SHALLOW_MAX_VELOCITY_PER_SEC = shallow_maxVelocityPerSec;
        SHALLOW_MAX_ACCELERATION_PER_SEC_SQUARED = shallow_maxAccelerationPerSecSquared;
        SHALLOW_TOLERANCE = shallow_tolerance;
        SHALLOW_MAX_RADIANS = shallow_maxRadians;

        DEEP_TOP_MOTOR_ID = deep_topMotorId;
        DEEP_BOTTOM_MOTOR_ID = deep_bottomMotorId;
        DEEP_CANCODER_ID = deep_cancoderId;

        DEEP_KP = deep_kP;
        DEEP_KI = deep_kI;
        DEEP_KD = deep_kD;

        DEEP_KS = deep_kS;
        DEEP_KG = deep_kG;
        DEEP_KV = deep_kV;

        DEEP_MAX_VELO_PER_SECOND = deep_maxVelocityPerSecond;
        DEEP_MAX_ACCEL_PER_SECOND = deep_maxAccelerationPerSecond;
    }
}
