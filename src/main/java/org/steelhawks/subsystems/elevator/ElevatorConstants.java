package org.steelhawks.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.steelhawks.Constants;

import java.util.Arrays;

public class ElevatorConstants {

    public enum State {
        L4(59.905784718904, 23.299634187195004, Units.rotationsToRadians(3.0)), // Before claw raise: 24.21
        L3(35.3237425930366, 13.956157208183564, Units.rotationsToRadians(2.0)), // Slightly too high: 14.394875713518857 // Before claw raise: 14.947108797157687
        L2(19.376478322177476, 8.308039947188632, Units.rotationsToRadians(1.0)), // Before claw raise: 9.10417597610128
        L1(11.3936423020206, 4.947855031325136, Units.rotationsToRadians(0.5)),
        HOME(0.0, 0.0, 0.0),

        // move elevator up so claw is not blocking the climb and cage
        PREPARE_CLIMB(0.0, 10.0, 0.0),
        // This is the "HOME" position the elevator goes to, before manually going down to the ACTUAL home position at the bottom bar
        HOME_ABOVE_BAR(0.0, 1.0, 0.0),

        // Algae Knockout Positions
        KNOCK_L2(0.0, 5.0, 0.0),
        KNOCK_L3(0.0, 12.038681223326511, 0.0);

        private final double alphaRadians;
        private final double omegaRadians;
        private final double hawkriderRadians;

        State(double alpha, double omega, double hawkrider) {
            this.alphaRadians = alpha;
            this.omegaRadians = omega;
            this.hawkriderRadians = hawkrider;
        }

        public Rotation2d getAngle() {
            double angleRads =
                switch (Constants.getRobot()) {
                    case ALPHABOT -> alphaRadians;
                    case OMEGABOT -> omegaRadians;
                    case HAWKRIDER -> hawkriderRadians;
                    default -> 0;
                };
            return new Rotation2d(angleRads);
        }
    }

    public static double CANCODER_OFFSET = 0;

    public static final int LIMIT_SWITCH_ID;
    public static final int LEFT_ID;
    public static final int RIGHT_ID;
    public static final int CANCODER_ID;

    public static final double GEAR_RATIO;

    public static final double CORAL_KS;
    public static final double CORAL_KG;
    public static final double CORAL_KV;
    public static final double CORAL_KA;

    public static final double CORAL_KP;
    public static final double CORAL_KI;
    public static final double CORAL_KD;

    public static final double NONE_KS;
    public static final double NONE_KG;
    public static final double NONE_KV;
    public static final double NONE_KA;

    public static final double NONE_KP;
    public static final double NONE_KI;
    public static final double NONE_KD;

    public static final double MAX_VELOCITY_PER_SEC;
    public static final double MAX_ACCELERATION_PER_SEC_SQUARED;
    public static final double MAX_JERK_PER_SEC_CUBED;

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
                CORAL_KS = 0.18;
                CORAL_KG = 0.18625;
                CORAL_KV = Arrays.stream(new double[]{
                    (2.0 - 1) / (10.593671321138238 - 4.652870525814727),
                }).average().orElse(0.0);
                CORAL_KA = 0.0;
                NONE_KS = CORAL_KS;
                NONE_KG = CORAL_KG;
                NONE_KV = CORAL_KV;
                NONE_KA = CORAL_KA;
                CORAL_KP = 2.6;
                CORAL_KI = 0.0;
                CORAL_KD = 0.01;
                NONE_KP = CORAL_KP;
                NONE_KI = CORAL_KI;
                NONE_KD = CORAL_KD;
                MAX_VELOCITY_PER_SEC = 100.0;
                MAX_ACCELERATION_PER_SEC_SQUARED = 110.0;
                MAX_JERK_PER_SEC_CUBED = 0.0;
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
                CORAL_KS = 0.15;
                CORAL_KG = 0.09;
                CORAL_KV = 0.6;
                CORAL_KA = 0.0;
                NONE_KS = CORAL_KS;
                NONE_KG = CORAL_KG;
                NONE_KV = CORAL_KV;
                NONE_KA = CORAL_KA;
                CORAL_KP = 9.1;
                CORAL_KI = 0.0;
                CORAL_KD = 0.002;
                NONE_KP = CORAL_KP;
                NONE_KI = CORAL_KI;
                NONE_KD = CORAL_KD;
                MAX_VELOCITY_PER_SEC = 5.2;
                MAX_ACCELERATION_PER_SEC_SQUARED = 8.0;
                MAX_JERK_PER_SEC_CUBED = 0.0;
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
                CORAL_KS = 0.23;
                CORAL_KG = 0.176;
                CORAL_KV = (((2.0 - 1.0) / (4.086524818927348 - 1.8346410223112268)) + ((1.0 - 0.5) / (1.8346410223112268 - 0.6381360077604268))) / 2.0;
                CORAL_KA = 0.0;
                NONE_KS = 0.0;
                NONE_KG = 0.0;
                NONE_KV = 0.0;
                NONE_KA = 0.0;
                CORAL_KP = 5.0;
                CORAL_KI = 0.0;
                CORAL_KD = 0.0;
                NONE_KP = 0.0;
                NONE_KI = 0.0;
                NONE_KD = 0.0;
                MAX_VELOCITY_PER_SEC = 40; // 35
                MAX_ACCELERATION_PER_SEC_SQUARED = 60; // was 70
                MAX_JERK_PER_SEC_CUBED = 120;
                TOLERANCE = 0.02;
                MANUAL_ELEVATOR_INCREMENT = 0.55;
                MAX_RADIANS = 24; // 24.663
            }
        }
    }
}
