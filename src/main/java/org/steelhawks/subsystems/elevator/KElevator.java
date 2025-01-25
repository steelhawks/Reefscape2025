package org.steelhawks.subsystems.elevator;

public final class KElevator {

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
    public static final int RIGHT_ID = 20;
    public static final int CANCODER_ID = 22;

    public static final double KS = 0;
    public static final double KG = 0.15;
    public static final double KV = 2.6;

    public static final double KP = 3.9;
    public static final double KI = 0;
    public static final double KD = 0.001;

    public static final double MAX_VELOCITY_PER_SEC = 5.2;
    public static final double MAX_ACCELERATION_PER_SEC_SQUARED = 8;

    public static final double TOLERANCE = 0.005;
    public static final double MANUAL_ELEVATOR_INCREMENT = 0.5;

    public static final double MAX_ROTATIONS = 3;
}
