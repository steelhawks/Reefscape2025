package org.steelhawks.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import org.steelhawks.util.ConstantsFactory;

public class ElevatorConstants {

    public enum State {
        L4(23.299634187195004),
        L3(12.988215331027725),
        L2(7.059379585849721),
        L1(4.947855031325136),
        HOME(0.0),

        // move elevator up so claw is not blocking the climb and cage
        PREPARE_CLIMB(10.0),
        // This is the "HOME" position the elevator goes to, before manually going down to the ACTUAL home position at the bottom bar
        HOME_ABOVE_BAR(1.0),
        BARGE_SCORE(24.0),

        // Algae Knockout Positions
        KNOCK_L2(5.0),
        KNOCK_L3(12.038681223326511);

        private final double omegaRadians;

        State(double omega) {
            this.omegaRadians = omega;
        }

        public Rotation2d getAngle() {
            return new Rotation2d(omegaRadians);
        }
    }

    public static final int LEFT_MOTOR_ID = 0;
    public static final int RIGHT_MOTOR_ID = 0;

    public static final double MAX_RADIANS = 24.0;
    public static final double TOLERANCE = 0.03;
    public static final double REDUCTION = 0.0;

    public static final double MAX_VELOCITY = 40.0;
    public static final double MAX_ACCELERATION = 70.0;
    public static final double MANUAL_ELEVATOR_INCREMENT = 0.65;

    public static final double KP = ConstantsFactory.value(7.0, 6.0);
    public static final double KI = 0.0;
    public static final double KD = ConstantsFactory.value(0.0, 0.4);

    public static final double[] kS = {
        ConstantsFactory.value(0.0, 0.2),
        0.0,
        0.0
    };

    public static final double[] kV = {
        0.0,
        0.0,
        0.0
    };

    public static final double[] kG = {
        0.0,
        0.0,
        0.0
    };

    public static final double[] kA = {
        0.0,
        0.0,
        0.0
    };

//    LIMIT_SWITCH_ID = 0;
//    LEFT_ID = 13;
//    RIGHT_ID = 14;
//    CANCODER_ID = 16; // 16 cancoder
//    GEAR_RATIO = 25;
//    KS = RobotBase.isReal() ? 0.23 : 0.1; // recalc prolly too high
//    KG = 0.166;
//    KV = (((2.0 - 1.0) / (4.086524818927348 - 1.8346410223112268)) + ((1.0 - 0.5) / (1.8346410223112268 - 0.6381360077604268))) / 2.0;
//    KP = RobotBase.isReal() ? 7.0 : 1.0; // 7
//    KI = 0.0;
//    KD = RobotBase.isReal() ? 0.0 : 0.01;
//    MAX_VELOCITY_PER_SEC = 50; // 40
//    MAX_ACCELERATION_PER_SEC_SQUARED = 40; // was 60
//    TOLERANCE = 0.03;
//    MANUAL_ELEVATOR_INCREMENT = 0.65;
//    MAX_RADIANS = 24; // 24.663

}
