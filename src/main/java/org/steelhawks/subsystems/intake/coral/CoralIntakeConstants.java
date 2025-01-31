package org.steelhawks.subsystems.intake.coral;

import org.steelhawks.util.LoggedTunableNumber;

public final class CoralIntakeConstants {

    public static final CoralIntakeConstants DEFAULT =
        new CoralIntakeConstants(
            0
        );

    public static final CoralIntakeConstants ALPHA = DEFAULT;
    
    public static final CoralIntakeConstants OMEGA = DEFAULT;


    public final int MOTOR_ID;

    public CoralIntakeConstants(
        int intakeMotorId
    ) {
        MOTOR_ID = intakeMotorId;
    }
}
