package org.steelhawks.subsystems.climb;

import edu.wpi.first.math.util.Units;
import org.steelhawks.Constants;

public final class ClimbConstants {

    public enum State {
        HOME(0, 0, 0),
        CLIMB(0, 0, 0);

        private final double alphaRadians;
        private final double omegaRadians;
        private final double hawkriderRadians;

        State(double alpha, double omega, double hawkrider) {
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
            Units.rotationsToRadians(3));

    public static final ClimbConstants OMEGA = DEFAULT;

    public static final ClimbConstants ALPHA =
        new ClimbConstants(
            21,
            1,
            8,
            0.005,
            0.5,
            Units.rotationsToRadians(3));

    public static final ClimbConstants HAWKRIDER = DEFAULT;

    public final int MOTOR_ID;

    public final double GEAR_RATIO;

    public final double MAX_VELOCITY_PER_SEC;
    public final double MAX_ACCELERATION_PER_SEC_SQUARED;

    public final double TOLERANCE;

    public final double MAX_RADIANS;

    public ClimbConstants(
        int motorId,
        double gearRatio,
        double maxVelocityPerSec,
        double maxAccelerationPerSecSquared,
        double tolerance,
        double maxRadians
    ) {
        MOTOR_ID = motorId;
        GEAR_RATIO = gearRatio;
        MAX_VELOCITY_PER_SEC = maxVelocityPerSec;
        MAX_ACCELERATION_PER_SEC_SQUARED = maxAccelerationPerSecSquared;
        TOLERANCE = tolerance;
        MAX_RADIANS = maxRadians;
    }
}
