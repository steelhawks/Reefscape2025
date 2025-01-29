package org.steelhawks.subsystems.elevator;

import org.steelhawks.util.LoggedTunableNumber;

public final class ElevatorConstants {

    public enum State {
        L4(3),
        L3(2),
        L2(1),
        L1(.5);

        State(double rotations) {
            this.rotations = rotations;
        }

        public double rotations;
    }

    public static final int LIMIT_SWITCH_ID = 0;
    public static final int LEFT_ID = 20;
    public static final int RIGHT_ID = 21;
    public static final int CANCODER_ID = 22;

    public static final LoggedTunableNumber KS =
        new LoggedTunableNumber("Elevator/KS", 0);
    public static final LoggedTunableNumber KG =
        new LoggedTunableNumber("Elevator/KG", 0.15);
    public static final LoggedTunableNumber KV =
        new LoggedTunableNumber("Elevator/KV", 2.6);

    public static final LoggedTunableNumber KP =
        new LoggedTunableNumber("Elevator/KP", 3.9);
    public static final LoggedTunableNumber KI =
        new LoggedTunableNumber("Elevator/KI", 0);
    public static final LoggedTunableNumber KD =
        new LoggedTunableNumber("Elevator/KD", 0.001);

    public static final LoggedTunableNumber MAX_VELOCITY_PER_SEC =
        new LoggedTunableNumber("Elevator/Max Velocity Per Sec", 5.2);
    public static final LoggedTunableNumber MAX_ACCELERATION_PER_SEC_SQUARED =
        new LoggedTunableNumber("Elevator/Max Acceleration Per Sec Squared", 8);

    public static final double TOLERANCE = 0.005;
    public static final double MANUAL_ELEVATOR_INCREMENT = 0.5;

    public static final double MAX_ROTATIONS = 3;
}
