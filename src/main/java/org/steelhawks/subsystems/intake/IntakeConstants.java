package org.steelhawks.subsystems.intake;

import org.steelhawks.Constants;
import org.steelhawks.util.LoggedTunableNumber;

public class IntakeConstants {

    public enum AlgaeIntakeState {
        HOME(0.0, 0.0, 3.0),
        INTAKE(0.0, 0.0, 2.0);

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

    public static final IntakeConstants DEFAULT =
        new IntakeConstants(
            16,
            .1,
            
            1,
            17,
            18,
            50,
            0,
            0.15,
            2.6,
            3.9,
            0,
            0.01,
            5.2,
            8,
            0.005,
            0.5,
            3);

    public static final IntakeConstants OMEGA = DEFAULT;
    public static final IntakeConstants ALPHA =
        new IntakeConstants(
            16,
            .3,

            1,
            18,
            17,
            50,
            0.37,
            0.4,
            2.6,
            3.9,
            0,
            0.01,
            5.2,
            8,
            0.005,
            0.5,
            3);

    public static final IntakeConstants HAWKRIDER = DEFAULT;


    // -------------------- CORAL --------------------
    public final int CORAL_INTAKE_MOTOR_ID;
    public final double CORAL_INTAKE_SPEED;


    // -------------------- ALGAE --------------------
    public final int ALGAE_LIMIT_SWITCH_ID;

    public final int ALGAE_INTAKE_MOTOR_ID;
    public final int ALGAE_PIVOT_MOTOR_ID;

    public final int ALGAE_CANCODER_ID;

    public final double ALGAE_KS;
    public final double ALGAE_KG;
    public final double ALGAE_KV;

    public final double ALGAE_KP;
    public final double ALGAE_KI;
    public final double ALGAE_KD;

    public final double ALGAE_MAX_VELOCITY_PER_SEC;
    public final double ALGAE_MAX_ACCELERATION_PER_SEC_SQUARED;

    public final double ALGAE_TOLERANCE;
    public final double ALGAE_MANUAL_PIVOT_INCREMENT;

    public final double ALGAE_MAX_RADIANS;

    public IntakeConstants(
        int coral_intakeMotorId,
        double coral_intakeSpeed,

        int algae_limitSwitchId,
        int algae_intakeMotorId,
        int algae_pivotMotorId,
        int algae_canCoderId,
        double algae_kS,
        double algae_kG,
        double algae_kV,
        double algae_kP,
        double algae_kI,
        double algae_kD,
        double algae_maxVelocityPerSec,
        double algae_maxAccelerationPerSecSquared,
        double algae_tolerance,
        double algae_manualPivotIncrement,
        double algae_maxRadians
    ) {
        CORAL_INTAKE_MOTOR_ID = coral_intakeMotorId;
        CORAL_INTAKE_SPEED = coral_intakeSpeed;

        ALGAE_LIMIT_SWITCH_ID = algae_limitSwitchId;
        ALGAE_INTAKE_MOTOR_ID = algae_intakeMotorId;
        ALGAE_PIVOT_MOTOR_ID = algae_pivotMotorId;
        ALGAE_CANCODER_ID = algae_canCoderId;
        ALGAE_KS = algae_kS;
        ALGAE_KG = algae_kG;
        ALGAE_KV = algae_kV;
        ALGAE_KP = algae_kP;
        ALGAE_KI = algae_kI;
        ALGAE_KD = algae_kD;
        ALGAE_MAX_VELOCITY_PER_SEC = algae_maxVelocityPerSec;
        ALGAE_MAX_ACCELERATION_PER_SEC_SQUARED = algae_maxAccelerationPerSecSquared;
        ALGAE_TOLERANCE = algae_tolerance;
        ALGAE_MANUAL_PIVOT_INCREMENT = algae_manualPivotIncrement;
        ALGAE_MAX_RADIANS = algae_maxRadians;
    }  
}