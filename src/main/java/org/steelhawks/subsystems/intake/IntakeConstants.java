package org.steelhawks.subsystems.intake;

import org.steelhawks.util.LoggedTunableNumber;

public class IntakeConstants {
    
    public enum AlgaeIntakeState {
        HOME(2),                    // Vertical
        INTAKE(1);                  

        AlgaeIntakeState(double rotations) {
            this.rotations = rotations;
        }

        public double rotations;
    }

    public static final IntakeConstants DEFAULT =
        new IntakeConstants(
            0, 
            
            0,
            20,
            21,
            22,
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
    public static final IntakeConstants ALPHA = DEFAULT;
    public static final IntakeConstants HAWKRIDER = DEFAULT;


    // ----------------------- CORAL -----------------------
    public final int CORAL_INTAKE_MOTOR_ID;


    // ----------------------- ALGAE -----------------------
    public final int ALGAE_LIMIT_SWITCH_ID;

    public final int ALGAE_INTAKE_MOTOR_ID;
    public final int ALGAE_PIVOT_MOTOR_ID;

    public final int ALGAE_CANCODER_ID;

    public final LoggedTunableNumber ALGAE_KS;
    public final LoggedTunableNumber ALGAE_KG;
    public final LoggedTunableNumber ALGAE_KV;

    public final LoggedTunableNumber ALGAE_KP;
    public final LoggedTunableNumber ALGAE_KI;
    public final LoggedTunableNumber ALGAE_KD;

    public final LoggedTunableNumber ALGAE_MAX_VELOCITY_PER_SEC;
    public final LoggedTunableNumber ALGAE_MAX_ACCELERATION_PER_SEC_SQUARED;

    public final double ALGAE_TOLERANCE;
    public final double ALGAE_MANUAL_PIVOT_INCREMENT;

    public final double ALGAE_MAX_ROTATIONS;

    public IntakeConstants(
        int coral_intakeMotorId,

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
        double algae_maxRotations
    ) {
        CORAL_INTAKE_MOTOR_ID = coral_intakeMotorId;

        ALGAE_LIMIT_SWITCH_ID = algae_limitSwitchId;
        ALGAE_INTAKE_MOTOR_ID = algae_intakeMotorId;
        ALGAE_PIVOT_MOTOR_ID = algae_pivotMotorId;
        ALGAE_CANCODER_ID = algae_canCoderId;
        ALGAE_KS = new LoggedTunableNumber("Algae Intake/KS", algae_kS);
        ALGAE_KG = new LoggedTunableNumber("Algae Intake/KG", algae_kG);
        ALGAE_KV = new LoggedTunableNumber("Algae Intake/KV", algae_kV);
        ALGAE_KP = new LoggedTunableNumber("Algae Intake/KP", algae_kP);
        ALGAE_KI = new LoggedTunableNumber("Algae Intake/KI", algae_kI);
        ALGAE_KD = new LoggedTunableNumber("Algae Intake/KD", algae_kD);
        ALGAE_MAX_VELOCITY_PER_SEC = new LoggedTunableNumber("Algae Intake/Max Velocity Per Sec", algae_maxVelocityPerSec);
        ALGAE_MAX_ACCELERATION_PER_SEC_SQUARED = new LoggedTunableNumber("Algae Intake/Max Acceleration Per Sec Squared", algae_maxAccelerationPerSecSquared);
        ALGAE_TOLERANCE = algae_tolerance;
        ALGAE_MANUAL_PIVOT_INCREMENT = algae_manualPivotIncrement;
        ALGAE_MAX_ROTATIONS = algae_maxRotations;
    }  
}