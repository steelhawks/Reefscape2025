package org.steelhawks.subsystems.intake;

import org.steelhawks.util.LoggedTunableNumber;

public class IntakeConstants {

    // Algae
    public static final int ALGAE_PIVOT_ID = 30;
    public static final int ALGAE_INTAKE_ID = 31;
    public static final int PIVOT_CANCODER_ID = 32;

    public static final LoggedTunableNumber ALGAE_KP =
        new LoggedTunableNumber("Intake/Algae/KP", 0);
    public static final LoggedTunableNumber ALGAE_KI =
        new LoggedTunableNumber("Intake/Algae/KI", 0);
    public static final LoggedTunableNumber ALGAE_KD =
        new LoggedTunableNumber("Intake/Algae/KD", 0);
    public static final LoggedTunableNumber ALGAE_MAX_VELOCITY_PER_SEC =
        new LoggedTunableNumber("Intake/Algae/MaxVelocityPerSec", 0);
    public static final LoggedTunableNumber ALGAE_MAX_ACCELERATION_PER_SEC_SQUARED =
        new LoggedTunableNumber("Intake/Algae/MaxAccelerationPerSecSquared", 0);

    public static final LoggedTunableNumber ALGAE_KS =
        new LoggedTunableNumber("Intake/Algae/KS", 0);
    public static final LoggedTunableNumber ALGAE_KG =
        new LoggedTunableNumber("Intake/Algae/KG", 0);
    public static final LoggedTunableNumber ALGAE_KV =
        new LoggedTunableNumber("Intake/Algae/KV", 0);

    public static final LoggedTunableNumber ALGAE_TOLERANCE =
        new LoggedTunableNumber("Intake/Algae/Tolerance", 0);

    // Coral
    public static final int CORAL_INTAKE_ID = 33;
    public static final int CORAL_PIVOT_ID = 34;
    public static final int CORAL_CANCODER_ID = 35;

    public static final LoggedTunableNumber CORAL_KP =
        new LoggedTunableNumber("Intake/Coral/KP", 0);
    public static final LoggedTunableNumber CORAL_KI =
        new LoggedTunableNumber("Intake/Coral/KI", 0);
    public static final LoggedTunableNumber CORAL_KD =
        new LoggedTunableNumber("Intake/Coral/KD", 0);

    public static final LoggedTunableNumber CORAL_MAX_VELOCITY_PER_SEC =
        new LoggedTunableNumber("Intake/Coral/MaxVelocityPerSec", 0);
    public static final LoggedTunableNumber CORAL_MAX_ACCELERATION_PER_SEC_SQUARED =
        new LoggedTunableNumber("Intake/Coral/MaxAccelerationPerSecSquared", 0);

    public static final LoggedTunableNumber CORAL_KS =
        new LoggedTunableNumber("Intake/Coral/KS", 0);
    public static final LoggedTunableNumber CORAL_KG =
        new LoggedTunableNumber("Intake/Coral/KG", 0);
    public static final LoggedTunableNumber CORAL_KV =
        new LoggedTunableNumber("Intake/Coral/KV", 0);

    public static final LoggedTunableNumber CORAL_TOLERANCE =
        new LoggedTunableNumber("Intake/Coral/Tolerance", 0);

}
