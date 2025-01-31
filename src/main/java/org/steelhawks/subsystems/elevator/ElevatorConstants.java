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

    public final int LIMIT_SWITCH_ID;
    public final int LEFT_ID;
    public final int RIGHT_ID;
    public final int CANCODER_ID;

    public final LoggedTunableNumber KS;
    public final LoggedTunableNumber KG;
    public final LoggedTunableNumber KV;

    public final LoggedTunableNumber KP;
    public final LoggedTunableNumber KI;
    public final LoggedTunableNumber KD;

    public final LoggedTunableNumber MAX_VELOCITY_PER_SEC;
    public final LoggedTunableNumber MAX_ACCELERATION_PER_SEC_SQUARED;

    public final double TOLERANCE;
    public final double MANUAL_ELEVATOR_INCREMENT;

    public final double MAX_ROTATIONS;

    public ElevatorConstants(
        int limitSwitchId,
        int leftMotorId,
        int rightMotorId,
        int canCoderId,
        double kS,
        double kG,
        double kV,
        double kP,
        double kI,
        double kD,
        double maxVelocityPerSec,
        double maxAccelerationPerSecSquared,
        double tolerance,
        double manualElev,
        double maxRotations
    ) {
        LIMIT_SWITCH_ID = limitSwitchId;
        LEFT_ID = leftMotorId;
        RIGHT_ID = rightMotorId;
        CANCODER_ID = canCoderId;
        KS = new LoggedTunableNumber("Elevator/KS", kS);
        KG = new LoggedTunableNumber("Elevator/KG", kG);
        KV = new LoggedTunableNumber("Elevator/KV", kV);
        KP = new LoggedTunableNumber("Elevator/KP", kP);
        KI = new LoggedTunableNumber("Elevator/KI", kI);
        KD = new LoggedTunableNumber("Elevator/KD", kD);
        MAX_VELOCITY_PER_SEC = new LoggedTunableNumber("Elevator/Max Velocity Per Sec", maxVelocityPerSec);
        MAX_ACCELERATION_PER_SEC_SQUARED = new LoggedTunableNumber("Elevator/Max Acceleration Per Sec Squared", maxAccelerationPerSecSquared);
        TOLERANCE = tolerance;
        MANUAL_ELEVATOR_INCREMENT = manualElev;
        MAX_ROTATIONS = maxRotations;
    }
}
