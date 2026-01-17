package org.steelhawks.subsystems.algae;

import org.steelhawks.Constants;

public class AlgaeIntakeConstants {

    public enum AlgaeIntakeState {
        // Keep in mind that all angle measurements listed below are in RADIANS!
        HOME(1.599829949 + 0.2945243112740431, 1.3744467859455347, 3.0), // HOME was previously 1.777936017374823
        INTAKE(0.20, 0.3604854851531256, 2.0),
        OUTTAKE((Math.PI / 1.777936017374823), 1.0, 0.0);

        // Algae Intake Arm's Lexan Component is Perfectly Horizontal: -0.2715145994557585
        // Algae Intake Arm's Lexan Component is Perfectly Vertical: 1.4741555371581012

        private final double alphaRadians;
        private final double omegaRadians;
        private final double hawkriderRadians;

        AlgaeIntakeState(double alpha, double omega, double hawkrider) {
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

    public static final int ALGAE_LIMIT_SWITCH_ID;
    public static final int ALGAE_INTAKE_MOTOR_ID;
    public static final int ALGAE_PIVOT_MOTOR_ID;
    public static final int ALGAE_CANCODER_ID;

    public static final double ALGAE_INTAKE_GEAR_RATIO;
    public static final double ALGAE_PIVOT_GEAR_RATIO;

    public static final double ALGAE_KS;
    public static final double ALGAE_KG;
    public static final double ALGAE_KV;

    public static final double ALGAE_KP;
    public static final double ALGAE_KI;
    public static final double ALGAE_KD;

    public static final double ALGAE_SPEED_MULTIPLIER;
    public static final double ALGAE_MAX_VELOCITY_PER_SEC;
    public static final double ALGAE_MAX_ACCELERATION_PER_SEC_SQUARED;

    public static final double ALGAE_TOLERANCE;
    public static final double ALGAE_MANUAL_PIVOT_INCREMENT;

    public static final double ALGAE_MAX_RADIANS;
    public static final double ALGAE_PIVOT_ZERO_OFFSET;

    static {
        switch (Constants.getRobot()) {
            case ALPHABOT, HAWKRIDER -> {
                ALGAE_LIMIT_SWITCH_ID = 1;
                ALGAE_INTAKE_MOTOR_ID = 16;
                ALGAE_PIVOT_MOTOR_ID = 17;
                ALGAE_CANCODER_ID = 18;
                ALGAE_INTAKE_GEAR_RATIO = 1.0;
                ALGAE_PIVOT_GEAR_RATIO = 1.0;
                ALGAE_KS = 0.0;
                ALGAE_KG = 0.0;
                ALGAE_KV = 0.0;
                ALGAE_KP = 0.0;
                ALGAE_KI = 0.0;
                ALGAE_KD = 0.0;
                ALGAE_SPEED_MULTIPLIER = 0.0;
                ALGAE_MAX_VELOCITY_PER_SEC = 0.0;
                ALGAE_MAX_ACCELERATION_PER_SEC_SQUARED = 0.0;
                ALGAE_TOLERANCE = 0.0;
                ALGAE_MANUAL_PIVOT_INCREMENT = 0.0;
                ALGAE_MAX_RADIANS = 2 * Math.PI;
                ALGAE_PIVOT_ZERO_OFFSET = 0.0;
            }
            default -> {
                ALGAE_LIMIT_SWITCH_ID = 1;
                ALGAE_INTAKE_MOTOR_ID = 16;
                ALGAE_PIVOT_MOTOR_ID = 17;
                ALGAE_CANCODER_ID = 18;
                ALGAE_INTAKE_GEAR_RATIO = 1.0;
                ALGAE_PIVOT_GEAR_RATIO = 1.0;
                ALGAE_KS = 0.4;
                ALGAE_KG = 0.235;
                ALGAE_KV = 0.45 * 5.0;
                ALGAE_KP = 1.0;
                ALGAE_KI = 0.0;
                ALGAE_KD = 0.0;
                ALGAE_SPEED_MULTIPLIER = 0.0;
                ALGAE_MAX_VELOCITY_PER_SEC = 0.0;
                ALGAE_MAX_ACCELERATION_PER_SEC_SQUARED = 0.0;
                ALGAE_TOLERANCE = 0.0;
                ALGAE_MANUAL_PIVOT_INCREMENT = 0.1;
                ALGAE_MAX_RADIANS = 2 * Math.PI;
                ALGAE_PIVOT_ZERO_OFFSET = 2.2135342769189803;
            }
        }
    }
}
