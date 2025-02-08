package org.steelhawks.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import org.steelhawks.Constants;
import org.steelhawks.util.LoggedTunableNumber;

public final class ElevatorConstants {

    public enum State {
        L4(51.0, 0.0, Units.rotationsToRadians(3.0)),
        L3(25.34243640242231, 0.0, Units.rotationsToRadians(2.0)),
        L2(14.353151436088366, 0.0, Units.rotationsToRadians(1.0)),
        L1(Units.rotationsToRadians(0.5), 0.0, Units.rotationsToRadians(0.5));

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

    public static final ElevatorConstants DEFAULT =
        new ElevatorConstants(
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
            Units.rotationsToRadians(3));

    public static final ElevatorConstants OMEGA = DEFAULT;

    public static final ElevatorConstants ALPHA =
        new ElevatorConstants(
            0,
            14,
            15,
            50,
            0.18,
            0.00625,
            2.6,
            3.9,
            0,
            0.01,
            3,
            5,
            Units.rotationsToRadians(0.005),
            0.25,
            59);

    public static final ElevatorConstants HAWKRIDER =
        new ElevatorConstants(
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
            Units.rotationsToRadians(.005),
            0.5,
            Units.rotationsToRadians(3));

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

    public final double MAX_RADIANS;

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
        double manualElevatorIncrement,
        double maxRadians
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
        MANUAL_ELEVATOR_INCREMENT = manualElevatorIncrement;
        MAX_RADIANS = maxRadians;
    }
}
