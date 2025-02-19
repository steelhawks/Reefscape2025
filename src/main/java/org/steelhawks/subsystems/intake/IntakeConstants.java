package org.steelhawks.subsystems.intake;

import org.steelhawks.Constants;

import edu.wpi.first.math.util.Units;

public class IntakeConstants {

    public enum AlgaeIntakeState {
        // Keep in mind that all angle measurements listed below are in RADIANS!
        HOME(1.599829949 + 0.2945243112740431, 0.0, 3.0), // HOME was previously 1.777936017374823
        INTAKE(0.20, 0.0, 2.0),
        OUTTAKE((Math.PI / 1.777936017374823), 0.0, 0.0);

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

    public enum SchlongState {
        // Keep in mind that all angle measurements listed below are in RADIANS!
        HOME(- Math.PI / 2, 0.0, 3.0), 
        ERECT(0, 0, 0);
        
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

    public static final IntakeConstants DEFAULT =
        new IntakeConstants(
            16,
            1,
            .1,
            .135,
            
            1,
            17,
            18,
            50,
            1,
            1,
            0,
            0.15,
            2.6,
            3.9,
            0,
            0.01,
            0.1,
            5.2,
            8,
            0.005,
            0.5,
            2 * Math.PI,
            
            23,
            24,
            3,
            
            1,
            1,
            
            0,
            0,
            0,
            0,
            0,
            0,
            1,
            5.2,
            8,
            Units.rotationsToRadians(0.005),
            0.5,
            Math.PI);

    public static final IntakeConstants OMEGA = DEFAULT;
    public static final IntakeConstants ALPHA =
        new IntakeConstants(
            16,
            1,
            .3,
            .135,

            1,
            18,
            17,
            50,
            10,
            22.64857645, // 25.17
            0.3525,
            0.37,
            0.71428571428571428571428571428571,
            // 3.9,
            // 0,
            // 0.01,
            0.05,
            0,
            0,
            0.1,
            5.2,
            8,
            0.005,
            0.5,
            3,
            
            23, 
            33,
            3,
            
            1,
            1,
            
            0,
            0,
            0,
            0,
            0,
            0,

            1,
            5.2,
            8,
            Units.rotationsToRadians(0.005),
            0.5,
            Math.PI);

    public static final IntakeConstants HAWKRIDER = DEFAULT;


    // -------------------- CORAL --------------------
    public final int CORAL_INTAKE_MOTOR_ID;

    public final double CORAL_INTAKE_GEAR_RATIO;

    public final double CORAL_SHOOT_SPEED;
    public final double CORAL_SECONDARY_SHOOT_SPEED;


    // -------------------- ALGAE --------------------
    public final int ALGAE_LIMIT_SWITCH_ID;
    public final int ALGAE_INTAKE_MOTOR_ID;
    public final int ALGAE_PIVOT_MOTOR_ID;
    public final int ALGAE_CANCODER_ID;

    public final double ALGAE_INTAKE_GEAR_RATIO;
    public final double ALGAE_PIVOT_GEAR_RATIO;

    public final double ALGAE_KS;
    public final double ALGAE_KG;
    public final double ALGAE_KV;

    public final double ALGAE_KP;
    public final double ALGAE_KI;
    public final double ALGAE_KD;

    public final double ALGAE_SPEED_MULTIPLIER;
    public final double ALGAE_MAX_VELOCITY_PER_SEC;
    public final double ALGAE_MAX_ACCELERATION_PER_SEC_SQUARED;

    public final double ALGAE_TOLERANCE;
    public final double ALGAE_MANUAL_PIVOT_INCREMENT;

    public final double ALGAE_MAX_RADIANS;

    public final int SCHLONG_SPIN_MOTOR_ID;
    public final int SCHLONG_PIVOT_MOTOR_ID;
    public final int SCHLONG_LIMIT_SWITCH_ID;

    public final double SCHLONG_SPIN_GEAR_RATIO;
    public final double SCHLONG_PIVOT_GEAR_RATIO;

    public final double SCHLONG_KS;
    public final double SCHLONG_KG;
    public final double SCHLONG_KV;

    public final double SCHLONG_KP;
    public final double SCHLONG_KI;
    public final double SCHLONG_KD;

    public final double SCHLONG_SPEED_MULTIPLIER;
    public final double SCHLONG_MAX_VELOCITY_PER_SEC;
    public final double SCHLONG_MAX_ACCELERATION_PER_SEC_SQUARED;
    public final double SCHLONG_TOLERANCE;
    public final double SCHLONG_MANUAL_PIVOT_INCREMENT;
    public final double SCHLONG_MAX_RADIANS;


    public IntakeConstants(
        int coral_intakeMotorId,
        double coral_intakeGearRatio,
        double coral_shootSpeed,
        double coral_secondaryShootSpeed,

        int algae_limitSwitchId,
        int algae_intakeMotorId,
        int algae_pivotMotorId,
        int algae_canCoderId,
        double algae_intakeGearRatio,
        double algae_pivotGearRatio,
        double algae_kS,
        double algae_kG,
        double algae_kV,
        double algae_kP,
        double algae_kI,
        double algae_kD,
        double algae_speedMultiplier,
        double algae_maxVelocityPerSec,
        double algae_maxAccelerationPerSecSquared,
        double algae_tolerance,
        double algae_manualPivotIncrement,
        double algae_maxRadians,

        int schlong_spinMotorId,
        int schlong_pivotMotorId,
        int schlong_limitSwitchId,

        double schlong_spinGearRatio,
        double schlong_pivotGearRatio,

        double schlong_kS,
        double schlong_kG,
        double schlong_kV,
        double schlong_kP,
        double schlong_kI,
        double schlong_kD,
        double schlong_speedMultiplier,
        double schlong_maxVelocityPerSec,
        double schlong_maxAccelerationPerSecSquared,
        double schlong_tolerance,
        double schlong_manualPivotIncrement,
        double schlong_maxRadians

    ) {
        CORAL_INTAKE_MOTOR_ID = coral_intakeMotorId;
        CORAL_INTAKE_GEAR_RATIO = coral_intakeGearRatio;
        CORAL_SHOOT_SPEED = coral_shootSpeed;
        CORAL_SECONDARY_SHOOT_SPEED = coral_secondaryShootSpeed;

        ALGAE_LIMIT_SWITCH_ID = algae_limitSwitchId;
        ALGAE_INTAKE_MOTOR_ID = algae_intakeMotorId;
        ALGAE_PIVOT_MOTOR_ID = algae_pivotMotorId;
        ALGAE_CANCODER_ID = algae_canCoderId;
        ALGAE_INTAKE_GEAR_RATIO = algae_intakeGearRatio;
        ALGAE_PIVOT_GEAR_RATIO = algae_pivotGearRatio;
        ALGAE_KS = algae_kS;
        ALGAE_KG = algae_kG;
        ALGAE_KV = algae_kV;
        ALGAE_KP = algae_kP;
        ALGAE_KI = algae_kI;
        ALGAE_KD = algae_kD;
        ALGAE_SPEED_MULTIPLIER = algae_speedMultiplier;
        ALGAE_MAX_VELOCITY_PER_SEC = algae_maxVelocityPerSec;
        ALGAE_MAX_ACCELERATION_PER_SEC_SQUARED = algae_maxAccelerationPerSecSquared;
        ALGAE_TOLERANCE = algae_tolerance;
        ALGAE_MANUAL_PIVOT_INCREMENT = algae_manualPivotIncrement;
        ALGAE_MAX_RADIANS = algae_maxRadians;

        SCHLONG_SPIN_MOTOR_ID = schlong_spinMotorId;
        SCHLONG_PIVOT_MOTOR_ID = schlong_pivotMotorId;
        SCHLONG_LIMIT_SWITCH_ID = schlong_limitSwitchId;

        SCHLONG_SPIN_GEAR_RATIO = schlong_spinGearRatio;
        SCHLONG_PIVOT_GEAR_RATIO = schlong_pivotGearRatio;

        SCHLONG_KS = schlong_kS;
        SCHLONG_KG = schlong_kG;
        SCHLONG_KV = schlong_kV;
        SCHLONG_KP = schlong_kP;
        SCHLONG_KI = schlong_kI;
        SCHLONG_KD = schlong_kD;
        SCHLONG_SPEED_MULTIPLIER = schlong_speedMultiplier;
        SCHLONG_MAX_VELOCITY_PER_SEC = schlong_maxVelocityPerSec;
        SCHLONG_MAX_ACCELERATION_PER_SEC_SQUARED = schlong_maxAccelerationPerSecSquared;
        SCHLONG_TOLERANCE = schlong_tolerance;
        SCHLONG_MANUAL_PIVOT_INCREMENT = schlong_manualPivotIncrement;
        SCHLONG_MAX_RADIANS = schlong_maxRadians;
    }  
}