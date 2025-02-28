package org.steelhawks.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import org.steelhawks.Constants;

import java.util.Arrays;

public final class ElevatorConstants {

    public enum State {
        L4(59.905784718904, 24.21, Units.rotationsToRadians(3.0)), // Omega L4 can only accurately shoot at 24.385 radians
        L3(35.3237425930366, 14.947108797157687, Units.rotationsToRadians(2.0)),
        L2(19.376478322177476, 9.10417597610128, Units.rotationsToRadians(1.0)),
        L1(11.3936423020206, 4.947855031325136, Units.rotationsToRadians(0.5)),
        HOME(0, 0, 0),

        // This is the "HOME" position the elevator goes to, before manually going down to the ACTUAL home position at the bottom bar
        HOME_ABOVE_BAR(0, 1, 0); 


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
            1,
            0,
            0.15,
            2.6,
            3.9,
            0,
            0.001,
            5.2,
            8,
            0.005,
            0.5,
            Units.rotationsToRadians(3));

    public static final ElevatorConstants OMEGA =
        new ElevatorConstants(
            0,
            13,
            14,
            16,
            25,
            0.23,
            0.175,
            (((2.0 - 1.0) / (4.086524818927348 - 1.8346410223112268)) + ((1.0 - 0.5) / (1.8346410223112268 - 0.6381360077604268))) / 2.0,
            5.5, // 2.75
            0,
            0.02, // 0.0145
            35,
            70,
            0.02,
            0.55,
            24.21235275598696);

    public static final ElevatorConstants ALPHA =
        new ElevatorConstants(
            0,
            14,
            15,
            -1,
            10,
            0.18,
            0.18625,
            Arrays.stream(new double[]{
                (2.0 - 1) / (10.593671321138238 - 4.652870525814727),
            }).average().orElse(0.0),
            2.6,
            0,
            0.01, // 0.126
            100,
            110,
            Units.rotationsToRadians(0.005),
            0.65, // 0.55
            60); // 60

    public static final ElevatorConstants HAWKRIDER =
        new ElevatorConstants(
        0,
        20,
        21,
        22,
        1,
        0.15,
        0.09,
        0.6,
        9.1,
        0,
        0.002,
        5.2,
        8,
        Units.rotationsToRadians(.005),
        0.5,
        18.5);

    public final int LIMIT_SWITCH_ID;
    public final int LEFT_ID;
    public final int RIGHT_ID;
    public final int CANCODER_ID;

    public final double GEAR_RATIO;

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
        double gearRatio,
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
        GEAR_RATIO = gearRatio;
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
