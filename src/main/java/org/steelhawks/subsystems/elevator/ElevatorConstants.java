package org.steelhawks.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import org.steelhawks.Constants;

import java.util.Arrays;

public class ElevatorConstants {

    public enum State {
        L4(59.905784718904, 23.299634187195004, Units.rotationsToRadians(3.0)), // Before claw raise: 24.21
        L3(35.3237425930366, 13.956157208183564, Units.rotationsToRadians(2.0)), // Slightly too high: 14.394875713518857 // Before claw raise: 14.947108797157687
        L2(19.376478322177476, 8.308039947188632, Units.rotationsToRadians(1.0)), // Before claw raise: 9.10417597610128
        L1(11.3936423020206, 4.947855031325136, Units.rotationsToRadians(0.5)),
        HOME(0, 0, 0),

        // This is the "HOME" position the elevator goes to, before manually going down to the ACTUAL home position at the bottom bar
        HOME_ABOVE_BAR(0, 1, 0),

        // Algae Knockout Positions
        KNOCK_L2(0, 5, 0),
        KNOCK_L3(0, 12.038681223326511, 0);

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

    public static double CANCODER_OFFSET = 0;

    public static final int LIMIT_SWITCH_ID;
    public static final int LEFT_ID;
    public static final int RIGHT_ID;
    public static final int CANCODER_ID;

    public static final double GEAR_RATIO;

    public static final double KS;
    public static final double KG;
    public static final double KV;

    public static final double KP;
    public static final double KI;
    public static final double KD;

    public static final double MAX_VELOCITY_PER_SEC;
    public static final double MAX_ACCELERATION_PER_SEC_SQUARED;

    public static final double TOLERANCE;
    public static final double MANUAL_ELEVATOR_INCREMENT;

    public static final double MAX_RADIANS;

    static {
        switch (Constants.getRobot()) {
            case ALPHABOT -> {
                LIMIT_SWITCH_ID = 0;
                LEFT_ID = 14;
                RIGHT_ID = 15;
                CANCODER_ID = -1;
                GEAR_RATIO = 10.0;
                KS = 0.18;
                KG = 0.18625;
                KV = Arrays.stream(new double[]{
                    (2.0 - 1) / (10.593671321138238 - 4.652870525814727),
                }).average().orElse(0.0);
                KP = 2.6;
                KI = 0.0;
                KD = 0.01;
                MAX_VELOCITY_PER_SEC = 100.0;
                MAX_ACCELERATION_PER_SEC_SQUARED = 110.0;
                TOLERANCE = Units.rotationsToRadians(0.005);
                MANUAL_ELEVATOR_INCREMENT = 0.65;
                MAX_RADIANS = 60;
            }
            case HAWKRIDER -> {
                LIMIT_SWITCH_ID = 0;
                LEFT_ID = 20;
                RIGHT_ID = 21;
                CANCODER_ID = 22;
                GEAR_RATIO = 1.0;
                KS = 0.15;
                KG = 0.09;
                KV = 0.6;
                KP = 9.1;
                KI = 0.0;
                KD = 0.002;
                MAX_VELOCITY_PER_SEC = 5.2;
                MAX_ACCELERATION_PER_SEC_SQUARED = 8.0;
                TOLERANCE = Units.rotationsToRadians(0.005);
                MANUAL_ELEVATOR_INCREMENT = 0.5;
                MAX_RADIANS = 18.5;
            }
            default -> {
                LIMIT_SWITCH_ID = 0;
                LEFT_ID = 13;
                RIGHT_ID = 14;
                CANCODER_ID = 16; // 16 cancoder
                GEAR_RATIO = 25;
                KS = 0.23;
                KG = 0.176;
                KV = (((2.0 - 1.0) / (4.086524818927348 - 1.8346410223112268)) + ((1.0 - 0.5) / (1.8346410223112268 - 0.6381360077604268))) / 2.0;
                KP = 5.0;
                KI = 0.0;
                KD = 0.0;
                MAX_VELOCITY_PER_SEC = 40; // 35
                MAX_ACCELERATION_PER_SEC_SQUARED = 60; // was 70
                TOLERANCE = 0.02;
                MANUAL_ELEVATOR_INCREMENT = 0.55;
                MAX_RADIANS = 24; // 24.663
            }
        }
    }
}
