package org.steelhawks.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import org.steelhawks.Constants;
import org.steelhawks.util.LoggedTunableNumber;

import java.util.Arrays;

public final class ElevatorConstants {

    public enum State {
        L4(58.36965635791532, 0.0, Units.rotationsToRadians(3.0)),
        L3(32.22141984761668, 0.0, Units.rotationsToRadians(2.0)),
        L2(16.452864338546235, 0.0, Units.rotationsToRadians(1.0)),
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
            0.18625,
            Arrays.stream(new double[]{
                (4.0 - 3.0) / (22.587560305458492 - 16.613011932801495),
                (3.0 - 2.0) / (16.613011932801495 - 10.530471312677351),
                (2.0 - 1.0) / (10.530471312677351 - 4.851674435924706),
            }).average().orElse(0.0),
            3.8,
            0,
            0.126,
            12,
            14,
            Units.rotationsToRadians(0.005),
            0.25,
            60);

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

    public final double KS;
    public final double KG;
    public final double KV;

    public final double KP;
    public final double KI;
    public final double KD;

    public final double MAX_VELOCITY_PER_SEC;
    public final double MAX_ACCELERATION_PER_SEC_SQUARED;

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
        KS = kS;
        KG = kG;
        KV = kV;
        KP = kP;
        KI = kI;
        KD = kD;
        MAX_VELOCITY_PER_SEC = maxVelocityPerSec;
        MAX_ACCELERATION_PER_SEC_SQUARED = maxAccelerationPerSecSquared;
        TOLERANCE = tolerance;
        MANUAL_ELEVATOR_INCREMENT = manualElevatorIncrement;
        MAX_RADIANS = maxRadians;
    }
}
